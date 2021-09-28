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
#ifndef STRUCTS_TAGS_H
#define STRUCTS_TAGS_H

#include <stdint.h>
#include <stdbool.h>

#define WRAPPER_TMR_MAX_EPC_BYTE_COUNT (62)
#define TMR_GEN2_MAX_PC_BYTE_COUNT (6)
#define DEFAULT_READ_TIMEOUT 1000

#define ERRO_ARRAY_NAO_INICIALIZADO 1
#define ERRO_READER_NAO_INICIALIZADO 2

#define CREATE_ERROR -1
#define CONNECT_ERROR -2

#define defaultURI "eapi:///dev/ttyACM0"

/**
 * Interface para acesso a API de controle do leitor rfid mercuryapi.
 * A implementação não pode ser usada de forma multi-thread!
 **/

typedef struct gen2_tagData{
     /** Length of the tag PC */
  uint8_t pcByteCount;
  /** Tag PC */
  uint8_t pc[TMR_GEN2_MAX_PC_BYTE_COUNT];
} gen2_tagData;

typedef struct tagData {
    uint8_t epc[WRAPPER_TMR_MAX_EPC_BYTE_COUNT];
    /** Length of the tag's EPC in bytes */
    uint8_t epcByteCount;
    /** Scrambled Brand Identifier of UCODE8 tag */
    uint8_t xepc[2];
    /** Tag CRC */
    uint16_t crc;
    /** Valor RSSI da leitura da Tag **/
    int32_t rssi;
    /** Antenna where the tag was read */
    uint8_t antenna;
    gen2_tagData gen2_data;
    
} tagData;

typedef struct tagsReadData {
    int tagCount;
    tagData* tagArray;
    int errorCode;
    
} tagsReadData;
/**
 * Struct com os campos de configuração dos Leitores RFID
 **/
typedef struct readerConfig{
    int modeloLeitor;//Leitor Acura ou SpitFire 0 ou 1
    int32_t readPower;// = 3150;/
    float tari;//= 2;//Valor int (0 [25microSec], 1 [12.525microSec], 2 [6.2525microSec])
    int blf;// = 250;//TValor int (250, 320 ou 640)
    int tagEncoding;// = 3;Valor int (0,1,2,3) (sqrt(M));
    int session ; // = 1//Valor hex (0x00, 0x01, 0x02, 0x03)
    bool dynamicQ ;
    int Q;
    int initialQ;
    int linkFrequency; 
    int linkProfile; //SPIT_FIRE linkProfile
    int target; //A, B ou AB
    int antennaTimeoutMS; //Tempo de ativação de cada antena (MS)
} readerConfig;


#endif