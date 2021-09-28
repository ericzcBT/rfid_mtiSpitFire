/* RFID_BT Copyright BrameTec (2020-2021)
 * This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>
 * */
#include <stdio.h>  // standard input / output functions
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <iostream>

#include "serial.h"
#include "leitorSpitFire.h"
#include "structsTags.h"

#define MAX_NUMBER_OF_TAGS 900

void testInventario();
bool testaConexaoDoLeitor(const char* uriPortaTeste);


int main(int argc, char **argv) {
    const char *port = argc > 1 ? argv[1] : "/dev/ttyUSB0";
    int testToRun = argc > 2 ? atoi(argv[2]) : 0;
    printf("Port: %s. Test: %d\n", port, testToRun);
    // speed_t baudrates[] = {B115200, B230400, B460800, B921600};
    //for (int i = 0; i < 4; i++) {
    speed_t defaultBaudRate = B115200;
    if(testaConexaoDoLeitor(port) >= 0){
        testInventario();
    }
    leitorSpitFire_destroyReader();
}

// Exemplo Para Verificar portas tty Linux e conectar. 
// O correto é definir um alias fixo para o device usando udev http://reactivated.net/writing_udev_rules.html
// bool reader_connectProbingSerialPorts() {
//     printf("Finding reader Serial port...\n");
//     int arqPortasSize = 0;
//     int readerOk = -1;
//     //Adiciona o prefixo eapi:// as portas tty para gerar a URI do leitor: "eapi:///dev/ttyACM0"
//     const char* lsDev = "ls /dev/ttyACM* | sed 's#.*#eapi://&#' > portasSerial.txt";
//     const char* lsDev2 = "ls /dev/ttyUSB* >> portasSerial.txt";
//     system(lsDev);
//     system(lsDev2);
//     FILE* f;
//     f = fopen("portasSerial.txt", "r");
//     if (f != NULL) {
//         fseek(f, 0, SEEK_END);
//         arqPortasSize = ftell(f);
//         fseek(f, 0, SEEK_SET);
//         //return -1;
//     }
//     printf("PORTAS: %d", arqPortasSize);
//     char portaTeste[30];
//     if (arqPortasSize > 0) {
//         arqPortasSize++;
//         char portasttyUSB[arqPortasSize];
//         fread(portasttyUSB, arqPortasSize, 1, f);
//         fclose(f);
//         portasttyUSB[arqPortasSize] = '\0';
//         int index = 0;
//         int lastIndex = 0;
//         for (; index < arqPortasSize - 1 && readerOk < 0; index++) {
//             if (portasttyUSB[index] == '\n' || portasttyUSB[index] == '\0') {
//                 strncpy(portaTeste, portasttyUSB + lastIndex, index - lastIndex);
//                 portaTeste[index - lastIndex] = '\0';
//                 printf("...Testando Serial:'%s'...\n", portaTeste);
//                 lastIndex = index + 1;
//                 readerOk = wrapper_createReader(portaTeste);
//                 if (readerOk == CREATE_ERROR) {
//                     printf("ERRO de Criação do Leitor.");
//                 }
//                 readerOk = wrapper_connectReader();
//                 if (readerOk == CONNECT_ERROR) {
//                     printf("ERRO de Conexão do Leitor: %s", wrapper_getErrorString());
//                 } else {
//                     printf("Leitor RFID conectado em %s", portaTeste);
//                 }
//             }
//         }
//     }
//     return readerOk >= 0;
// }

bool testaConexaoDoLeitor(const char* uriPortaTeste) {
    bool readerOk = leitorSpitFire_createReader(uriPortaTeste);
    if (readerOk == CREATE_ERROR) {
        printf("ERRO de Criação do Leitor.");
    }
    readerOk = leitorSpitFire_testConnection();
    if (readerOk == CONNECT_ERROR) {
        printf("ERRO de Conexão do Leitor: ");
    } else {
        printf("Leitor RFID conectado em %s", uriPortaTeste);
    }
    return readerOk;
}

//Exemplo de conversão HEX para String (sdk mercuryapi-1.31.3.36)
static char hexchars[] = "0123456789ABCDEF";
void bytesToHex(const uint8_t *bytes, uint32_t size, char *hex) {
    if (hex == NULL) {
        printf("Ponteiro de string não inicializado!");
        return;
    }
    while (size--) {
        *hex++ = hexchars[*bytes >> 4];  //shift
        *hex++ = hexchars[*bytes & 15];  //&F
        bytes++;
    }
    *hex = '\0';
}

void testInventario() {
    //Configurações default
    readerConfig configsBd;
    leitorSpitFire_alteraConfiguracoes(configsBd);
    tagsReadData readInfo;
    int maxTagsTest = 900;
    int antenasTeste[]={1};
    int testTimeout = 0;
    bool readOk = leitorSpitFire_readTagInventory(&readInfo, maxTagsTest, testTimeout, antenasTeste, 1, configsBd);
    if(readOk){
        printf("Read %d tags\n",readInfo.tagCount);
        for (int currentTag = 0; currentTag < readInfo.tagCount; currentTag++) {
            //printf("Writing tag\n");
            char cHexString[(WRAPPER_TMR_MAX_EPC_BYTE_COUNT * 2) + 1];
            tagData tag = readInfo.tagArray[currentTag];
            bytesToHex(tag.epc, tag.epcByteCount, cHexString);
            //std::string hexString = std::string(cHexString);
            printf("EPC: %s\n",cHexString);
        }
    }
}
