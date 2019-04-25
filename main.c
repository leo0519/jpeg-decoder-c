#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<math.h>
#define CONC(x, y) (((int)x << 8) + y)
#define DEBUG

typedef unsigned char uchar;

typedef struct BinaryTree{
    struct BinaryTree *child[2];
    uchar value;
} BinaryTree;

typedef struct{
    FILE *fp;
    uchar buf;
    int count;
} BitStream;

typedef struct{
    int bIndex[4]; //block index in each MCU
    int nMCU, nBlock, nBlkPerComp[4]; // number of MCUs
    int hMCU, wMCU; // height x width per MCU
    int hSub[4], wSub[4]; //Subsampling propotion
    int qIndex[4]; //quantization table index for components
    int height, width, component; //height x width x component of pixels
    int hIndex[4][2]; //huffman table index [index][AC/DC]
    int qTable[2][64]; //quantization table [index]
    BinaryTree *hTable[2][4]; //huffman table [DC/AC][index]
    int *mcus; //decoding MCUs
} JPEG;

//handle bit stream
int get_bit(BitStream *stream){
    if(stream->count == 8){
        fread(&stream->buf, 1, 1, stream->fp);
        if(stream->buf == 0xFF){
            fread(&stream->buf, 1, 1, stream->fp);
            if(stream->buf == 0)stream->buf = 0xFF;
            else printf("0xFF without 0x00!!\n");
        }
        stream->count = 0;
    }
    return (stream->buf & 0x80) != 0;
}

void shift_next(BitStream *stream){
    stream->count++;
    stream->buf <<= 1;
}

//buffer to DQT
void buf2dqt(uchar *buf, int size, JPEG *jpeg){
    for(int i = 0, deep, id, *table; i < size; i += 1 + (deep + 1) * 64){
        deep = buf[i] >> 4;
        id = buf[i] & 0xF;
        table = jpeg->qTable[id];
        for(int j = 0; j < 64; j++){
            table[j] = deep ? CONC(buf[i + 2 * j + 1], buf[i + 2 * j]) : buf[i + j + 1];
        }
        #ifdef DEBUG
        printf("deep %d - idx %d\n", deep, id);
        #endif
    }
}

//buffer to SOF0
void buf2sof0(uchar *buf, JPEG *jpeg){
    jpeg->height = CONC(buf[1], buf[2]);
    jpeg->width = CONC(buf[3], buf[4]);
    jpeg->component = buf[5];
    int hMax, wMax;
    hMax = wMax = jpeg->nBlock = 0;
    for(int i = 0; i < jpeg->component; i++){
        jpeg->wSub[i] = buf[7 + i * 3] >> 4;
        jpeg->hSub[i] = buf[7 + i * 3] & 0xF;
        jpeg->qIndex[i] = buf[8 + i * 3];
        jpeg->bIndex[i] = jpeg->nBlock;
        jpeg->nBlkPerComp[i] = jpeg->hSub[i] * jpeg->wSub[i];
        jpeg->nBlock += jpeg->nBlkPerComp[i];
        if(jpeg->hSub[i] > hMax)hMax = jpeg->hSub[i];
        if(jpeg->wSub[i] > wMax)wMax = jpeg->wSub[i];
    }
    jpeg->hMCU = hMax * 8;
    jpeg->wMCU = wMax * 8;
    jpeg->nMCU = ((jpeg->height + jpeg->hMCU - 1) / jpeg->hMCU) * ((jpeg->width + jpeg->wMCU - 1) / jpeg->wMCU);
    #ifdef DEBUG
    printf("precision %d - height %d - width %d - components %d\n", buf[0], jpeg->height, jpeg->width, jpeg->component);
    for(int i = 0; i < jpeg->component; i++){
        printf("component id %d - 4:%d:%d - quant_table %d\n", i + 1, jpeg->wSub[i], jpeg->hSub[i], jpeg->qIndex[i]);
    }
    printf("vMax %d - hMax %d\n", hMax, wMax);
    #endif
}

//buffer to DHT
void buf2dht(uchar *buf, int size, JPEG *jpeg){
    while(size > 0){
        int kind = buf[0] >> 4;
        int idx = buf[0] & 0xF;
        jpeg->hTable[kind][idx] = (BinaryTree *)calloc(1, sizeof(BinaryTree));
        BinaryTree *root = jpeg->hTable[kind][idx], *temp;
        int cur = 17;
        size -= 17;
        #ifdef DEBUG
        printf("%s %d\n", kind ? "AC" : "DC", idx);
        #endif
        for(int i = 1, code = 0; i <= 16; i++, code <<= 1){
            for(int j = 0; j < buf[i]; j++){
                temp = root;
                for(int k = 0; k < i; k++){
                    int bit = (code >> (i - k - 1)) & 1;
                    if(temp->child[bit] == NULL)temp->child[bit] = (BinaryTree *)calloc(1, sizeof(BinaryTree));
                    temp = temp->child[bit];
                }
                temp->value = buf[cur++];
                code++;
            }
            size -= buf[i];
        }
        buf += cur;
    }
}

//buffer to SOS
void buf2sos(uchar *buf, JPEG *jpeg){
    for(int i = 0; i < buf[0]; i++){
        jpeg->hIndex[i][0] = buf[2 * (i + 1)] >> 4;
        jpeg->hIndex[i][1] = buf[2 * (i + 1)] & 0xF;
        printf("component %d - DC %d - AC %d\n", i, jpeg->hIndex[i][0], jpeg->hIndex[i][1]);
    }
}

int ssss(int code, int length){
    int interval = 1 << (length - 1);
    if(code < interval)code += (-2 * interval + 1);
    return code;
}

//preprocess MCU
void readMCU(FILE *fp, JPEG *jpeg){
    BitStream stream = {.fp = fp, .count = 8};
    int nMCU = jpeg->nMCU;
    int component = jpeg->component;
    printf("# of MCU - %d\n", nMCU);
    printf("# of block per MCU - %d\n", jpeg->nBlock);
    jpeg->mcus = (int *)malloc(sizeof(int) * 64 * jpeg->nBlock * nMCU);
    int *mcus = jpeg->mcus;
    int dataIdx = 0;
    uchar weight;
    for(int i = 0; i < nMCU; i++){
        for(int j = 0; j < component; j++){
            int *hIndex = jpeg->hIndex[j];
            int nBlkPerComp = jpeg->nBlkPerComp[j];
            for(int k = 0; k < nBlkPerComp; k++){
                for(int l = 0; l < 64; l++){
                    BinaryTree *temp = jpeg->hTable[l != 0][hIndex[l == 0]];
                    uchar weight;
                    while(temp != NULL){
                        int bit = get_bit(&stream);
                        if(temp->child[bit] != NULL)shift_next(&stream);
                        else weight = temp->value;
                        temp = temp->child[bit];
                    }
                    if(l == 0){
                        int code = 0;
                        for(int m = 0; m < weight; m++){
                            code |= get_bit(&stream);
                            shift_next(&stream);
                            if(m < weight - 1)code <<= 1;
                        }
                        mcus[dataIdx++] = ssss(code, (int)weight);
                    }
                    else{
                        if(weight == 0){
                            for(; l < 63; l++){
                                mcus[dataIdx++] = 0;
                            }
                        }
                        int zeros = weight >> 4;
                        for(int m = 0; m < zeros; l++, m++){
                            mcus[dataIdx++] = 0;
                        }
                        int rle = weight & 0xF;
                        int code = 0;
                        for(int m = 0; m < rle; m++){
                            code |= get_bit(&stream);
                            shift_next(&stream);
                            if(m < rle - 1)code <<= 1;
                        }
                        mcus[dataIdx++] = ssss(code, rle);
                    }
                }
            }
        }
    }
}

void process(JPEG *jpeg, char *filepath){
    int nMCU = jpeg->nMCU;
    int nComponent = jpeg->component;
    int height = jpeg->height;
    int width = jpeg->width;
    int nBlock = jpeg->nBlock;
    int *data = jpeg->mcus;
    int *quant = jpeg->qIndex;
    int *nBlkPerComp = jpeg->nBlkPerComp;
    printf("nMCU %d - nComponent %d - nBlock %d\n", nMCU, nComponent, nBlock);
    int pred[4] = {0};
    int dataIdx = 0;
    int zigzagIdx[64] = {0, 1, 5, 6, 14, 15, 27, 28, 2, 4, 7, 13, 16, 26, 29, 42, 3, 8, 12, 17, 25, 30, 41, 43, 9, 11, 18, 24, 31, 40, 44, 53, 10, 19, 23, 32, 39, 45, 52, 54, 20, 22, 33, 38, 46, 51, 55, 60, 21, 34, 37, 47, 50, 56, 59, 61, 35, 36, 48, 49, 57, 58, 62, 63};
    //do DPCM
    for(int i = 0; i < nMCU; i++){
        for(int j = 0; j < nComponent; j++){
            for(int k = 0; k < nBlkPerComp[j]; k++){
                data[dataIdx] += pred[j];
                pred[j] = data[dataIdx];
                dataIdx += 64;
            }
        }
    }
    printf("DCPM finish...\n");
    //do Quantization
    dataIdx = 0;
    for(int i = 0; i < nMCU; i++){
        for(int j = 0; j < nComponent; j++){
            int *qTable = jpeg->qTable[jpeg->qIndex[j]];
            for(int k = 0; k < nBlkPerComp[j]; k++){
                for(int l = 0; l < 64; l++){
                    data[dataIdx + l] *= qTable[l];
                }
                dataIdx += 64;
            }
        }
    }
    printf("Quantization finish...\n");
    //do IDCT
    double cosine[4096];
    #pragma omp parallel for
    for(int i = 0; i < 64; i++){
        for(int j = 0; j < 64; j++){
            int a = i / 8, b = i % 8, c = j / 8, d = j % 8;
            cosine[i * 64 + j] = cos((2 * a + 1) * b * 3.14159265359 / 16) * cos((2 * c + 1) * d * 3.14159265359 / 16);
        }
    }
    double temp[64];
    #pragma omp parallel for private(temp)
    for(int i = 0; i < nMCU * nBlock; i++){
        for(int j = 0; j < 64; j++){
            temp[j] = (double)data[i * 64 + zigzagIdx[j]];
        }
        for(int j = 0; j < 8; j++){
            temp[j] /= 1.41421356237;
            temp[j * 8] /= 1.41421356237;
        }
        for(int j = 0; j < 8; j++){
            for(int k = 0; k < 8; k++){
                int idx = i * 64 + j * 8 + k;
                double sum = 0;
                for(int l = 0; l < 8; l++){
                    for(int m = 0; m < 8; m++){
                        sum += temp[l * 8 + m] * cosine[j * 512 + l * 64 + k * 8 + m];
                    }
                }
                sum /= 4;
                data[idx] = (int)(round(sum)) + 128;
                if(data[idx] > 255)data[idx] = 255;
                if(data[idx] < 0)data[idx] = 0;
            }
        }
    }
    printf("IDCT finish...\n");
    //do Subsampling
    double yCbCr[height][width][3];
    int nHorizMCU = (width + jpeg->wMCU - 1) / jpeg->wMCU;
    int nVertiMCU = (height + jpeg->hMCU - 1) / jpeg->hMCU;
    for(int i = 0; i < jpeg->component; i++){
        int wExtend = jpeg->wMCU / jpeg->wSub[i] / 8;
        int hExtend = jpeg->hMCU / jpeg->hSub[i] / 8;
        #pragma omp parallel for
        for(int j = 0; j < height; j++){
            for(int k = 0; k < width; k++){
                int MCUIdx = j / jpeg->hMCU * nHorizMCU + k / jpeg->wMCU;
                int hPixel = (j % jpeg->hMCU) / hExtend;
                int wPixel = (k % jpeg->wMCU) / wExtend;
                int blockIdx = (hPixel / 8) * jpeg->wSub[i] + (wPixel / 8);
                blockIdx += jpeg->bIndex[i];
                hPixel %= 8;
                wPixel %= 8;
                yCbCr[j][k][i] = (double)data[(MCUIdx * nBlock + blockIdx) * 64 + hPixel * 8 + wPixel];
            }
        }
    }
    printf("Subsampling finish...\n");
    //do yCbCr to RGB
    double ttemp[4];
    #pragma omp parallel for private(ttemp)
    for(int i = 0; i < height; i++){
        for(int j = 0; j < width; j++){
            memcpy(ttemp, yCbCr[i][j], sizeof(double) * nComponent);
            ttemp[1] -= 128;
            ttemp[2] -= 128;
            yCbCr[i][j][0] = ttemp[0] + 1.402 * ttemp[2];
            yCbCr[i][j][1] = ttemp[0] - 0.344 * ttemp[1] - 0.714 * ttemp[2];
            yCbCr[i][j][2] = ttemp[0] + 1.772 * ttemp[1];
            for(int k = 0; k < jpeg->component; k++){
                if(yCbCr[i][j][k] > 255)yCbCr[i][j][k] = 255;
                if(yCbCr[i][j][k] < 0)yCbCr[i][j][k] = 0;
            }
        }
    }
    //write to BMP
    int filesize = 54 + 3 * height * width;
    uchar *img = (unsigned char *)malloc(filesize - 54);
    #pragma omp parallel for
    for(int i = 0; i < width; i++){
        for(int j = 0; j < height; j++){
            for(int k = 0; k < 3; k++){
                img[(j * width + i) * 3 + k] = (uchar)yCbCr[j][i][2 - k];
            }
        }
    }
    uchar bmpfileheader[14] = {'B', 'M', 0, 0, 0, 0, 0, 0, 0, 0, 54, 0, 0, 0};
    uchar bmpinfoheader[40] = {40, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 24, 0};
    uchar bmppad[3] = {0};
    memcpy(bmpfileheader + 2, &filesize, 4);
    memcpy(bmpinfoheader + 4, &width, 4);
    memcpy(bmpinfoheader + 8, &height, 4);
    int length = strlen(filepath);
    filepath[length - 3] = 'b';
    filepath[length - 2] = 'm';
    filepath[length - 1] = 'p';
    FILE *f = fopen(filepath, "wb");
    fwrite(bmpfileheader, 1, 14, f);
    fwrite(bmpinfoheader, 1, 40, f);
    for(int i = 0; i < height; i++){
        fwrite(img + (width * (height - i - 1) * 3), 3, width, f);
        fwrite(bmppad, 1, (4 - (width * 3) % 4) % 4, f);
    }
    fclose(f);
    printf("Write to BMP finish...\n");
}

JPEG *readjpeg(char *filepath){
    FILE *fp = fopen(filepath, "rb");
    printf("%d\n", (int)strlen(filepath));
    JPEG *jpeg = (JPEG *)malloc(sizeof(JPEG));
    uchar buf[65536];
    int size;
    while(fread(buf, 1, 2, fp)){
        if(buf[0] == 0xFF){
            switch(buf[1]){
                case 0xD8:
                    printf("[SOI]\n");
                    break;
                case 0xE0:
                    fread(buf, 1, 2, fp);
                    size = CONC(buf[0], buf[1]) - 2;
                    fread(buf, 1, size, fp);
                    printf("[APP0] %d bytes\n", size);
                    break;
                case 0xDB:
                    fread(buf, 1, 2, fp);
                    size = CONC(buf[0], buf[1]) - 2;
                    fread(buf, 1, size, fp);
                    printf("[DQT] %d bytes\n", size);
                    buf2dqt(buf, size, jpeg);
                    break;
                case 0xC0:
                    fread(buf, 1, 2, fp);
                    size = CONC(buf[0], buf[1]) - 2;
                    fread(buf, 1, size, fp);
                    printf("[SOF0] %d bytes\n", size);
                    buf2sof0(buf, jpeg);
                    break;
                case 0xC4:
                    fread(buf, 1, 2, fp);
                    size = CONC(buf[0], buf[1]) - 2;
                    fread(buf, 1, size, fp);
                    printf("[DHT] %d bytes\n", size);
                    buf2dht(buf, size, jpeg);
                    break;
                case 0xDD:
                    fread(buf, 1, 2, fp);
                    size = CONC(buf[0], buf[1]) - 2;
                    fread(buf, 1, size, fp);
                    printf("DRI %d bytes\n", size);
                    break;
                case 0xDA:
                    fread(buf, 1, 2, fp);
                    size = CONC(buf[0], buf[1]) - 2;
                    fread(buf, 1, size, fp);
                    printf("[SOS] %d bytes\n", size);
                    buf2sos(buf, jpeg);
                    printf("[MCU]\n");
                    readMCU(fp, jpeg);
                    process(jpeg, filepath);
                    break;
                case 0xD9:
                    printf("[EOI]\n");
                    break;
                default:
                    fread(buf, 1, 2, fp);
                    size = CONC(buf[0], buf[1]) - 2;
                    printf("[Other Headers] %d bytes\n", size);
                    fread(buf, 1, size, fp);
            }
        }
    }
    fclose(fp);
    return jpeg;
}

int main(int argc, char *argv[]){
    JPEG *jpeg = readjpeg(argv[1]);
    return 0;
}