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
#ifndef H_SPIT_FIRE
#define H_SPIT_FIRE
#include <stdint.h>

#include <functional>
#include <string>

#include "structsTags.h"
#include "serial.h"
#include "util.h"

namespace spitFire {

const uint8_t COMMAND_BEGIN = 0x00;       //0x0000,
const uint8_t COMMAND_END = 0x01;         //0x0001,
const uint8_t INVENTORY_RESPONSE = 0x05;  //0x0005,
const uint8_t TAG_ACESS = 0x06;           //0x0006,
const uint8_t COMMAND_WORK = 0x0E;        //0x000E

const uint8_t COMMAND_SET_DEVICEID = 0x00;
const uint8_t COMMAND_GET_DEVICEID = 0x01;
const uint8_t COMMAND_SET_OPERATIONMODE = 0x02;
const uint8_t COMMAND_GET_OPERATIONMODE = 0x03;
const uint8_t COMMAND_SET_CURRENTLINKPROFILE = 0x04;
const uint8_t COMMAND_GET_CURRENTLINKPROFILE = 0x05;
const uint8_t COMMAND_RADIO_WRITE_REGISTER = 0x06;
const uint8_t COMMAND_RADIO_READ_REGISTER = 0x07;
const uint8_t COMMAND_RADIO_WRITE_BANKEDREGISTER = 0x08;
const uint8_t COMMAND_RADIO_READ_BANKEDREGISTER = 0x09;
const uint8_t COMMAND_RADIO_READ_REGISTERINFO = 0x0A;
const uint8_t COMMAND_ANTENNA_PORT_SETSTATE = 0x10;
const uint8_t COMMAND_ANTENNA_PORT_GETSTATE = 0x11;
const uint8_t COMMAND_ANTENNA_PORT_SETCONFIGURATION = 0x12;
const uint8_t COMMAND_ANTENNA_PORT_GETCONFIGURATION = 0x13;
const uint8_t COMMAND_ANTENNA_PORT_SETSENSETHRESHOLD = 0x14;
const uint8_t COMMAND_ANTENNA_PORT_GETSENSETHRESHOLD = 0x15;

enum statusMessage {
    RFID_STATUS_OK = 0x00,
    RFID_ERROR_INVALID_PARAMETER = 0xF0,
    RFID_ERROR_MODULE_FAILURE = 0xFF,
    FALHA_CONTROLADOR = 0x01
};

//Modulo SpitFire transmite palavras em ordem Little-Endian.
//16BytesTotal
//FEDCBA9876543210
//size_t numBytes = 16;
struct commandPacket {
    //0x4D 54 49 43
    uint8_t header[4] = {0x4D, 0x54, 0x49, 0x43};
    uint8_t deviceID = 0xFF;
    uint8_t command;
    uint8_t parameters[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t checksum[2];
};
//size_t numBytes = 16;
struct responsePacket {
    uint8_t header[4];
    uint8_t deviceID;
    uint8_t command;
    uint8_t returnData[8];
    uint8_t checksum[2];
};

// size_t numBytes = 14;
struct commonPacketFields {
    uint8_t header[4];
    uint8_t pktRelationNumber;    //Numero de report packets
    uint8_t pktRelationSequence;  //Sequencia de report packets. Se == RelationNumber então acabou a transmissão de pacotes
    uint8_t reportVersion;
    uint8_t reportFlags;
    uint8_t reportType[2];
    uint8_t reportInfoLength[2];
    uint8_t reportSequence[2];
};
//size_t= 10
struct commandBeginPacket {
    uint8_t command[4];
    uint8_t msCounter[4];  //Contador em MS do inicio da operação
    uint8_t checksum[2];
};
//size_t= 10
struct commandEndPacket {
    uint8_t msCounter[4];  //Contador em MS do final da operação
    uint8_t status[4];     //Contador em MS do final da operação
    uint8_t checksum[2];
};
//Pacote resposta de uma operação de Inventário de Tags
//size_t= 50
struct inventoryResponsePacket {
    uint8_t counter[4];  //MS quando a tag foi lida OU numero de vezes que a tag foi inventariada(Varia de acordo com o modo de operação)
    uint8_t nbRSSI;      //RSSI narrowband da leitura da tag
    /* Fórmula para converter em DB:
        Exponent = bits[7:3], Mantissa = bits[2:0], Mantissa_size= 3;
            20 * log10 (2^Exponent * (1+Mantissa / 2^Matissa_Size))
        Exemplo: valor 0x48
        Exponent = 9, Mantissa = 0
            20 * log10 (2^9 * (1+0 / 2^3)) = 54.19
    */
    uint8_t wbRSSI;  //RSSI wideband da leitura da tag
    /* Fórmula para converter em DB:
        Exponent = bits[7:4], Mantissa = bits[3:0], Mantissa_size= 4;
            20 * log10 (2^Exponent * (1+Mantissa / 2^Matissa_Size))
        Exemplo: valor 0x48
        Exponent = 4, Mantissa = 8
            20 * log10 (2^4 * (1+8 / 2^4)) = 54.19
    */
    uint8_t anaControl[2];   ///Registrador de GainControl no momento da medição de RSSI
    uint8_t rssi[2];         //EPC RSSI ajustado pela calibração. Unidade = décimos de dBm
    uint8_t logic_ant[2];    //Porta lógica da antena
    uint8_t *inventoryData;  //Tamanho do vetor = (reportInfoLength - 3) * 4. Deve ser multiplo de 32bits
    //Numero de bytes válidos = ((reportInfoLength - 3) * 4) - (numero de padding bytes na FLAG de Reports)
    uint8_t *padding;  //Tamanho do vetor = Número de Bytes necessário para que o pacote contenha 64 bytes
    uint8_t checksum[2];
};
//Pacote resposta de uma operação de Gravação de Tags
//size_t= 50
struct tagAccessPacket {
    uint8_t counter[4];  //MS quando a tag foi lida
    uint8_t command;
    uint8_t tagErrorCode;        //0=erro, 03=memoria nao existe na tag, 04=memoria bloqueada, 0B=tag sem força para gravar, 0F= tag nao suporta cod ERRo
    uint8_t moduleErrorCode[2];  //
    uint8_t writeCount[2];       //Numero de palavras escritas na Tag
    uint8_t reserved[2];         //reservado. Ler sempre como 0
    uint8_t *accessData;         // Tamanho = (reportInfoLentgh-3)*4
                                 //Numero de bytes válidos = ((reportInfoLength - 3) * 4) - (numero de padding bytes na FLAG de Reports)
    uint8_t *padding;            //Tamanho do vetor = Número de Bytes necessário para que o pacote contenha 64 bytes
    uint8_t checksum[2];
};

//Pacote resposta indicando que uma operação está EM CURSO
//size_t= 10
struct commandWorkPacket {
    uint8_t msCounter[4];  //MS quando a operação finalizou
    uint8_t reservd[4];    //Reservado. Ler como 0
    uint8_t checksum[2];
};

/*Pacotes de Resposta do Sensor.
 Incluem obrigatoriamente todos os campos Comuns (commonPacketFields).
 Incluem 1 e somente 1 conjunto de campos do tipo da resposta (commandBegin OU commandEnd OU .....)    
*/
struct reportPacket {
    commonPacketFields common;
    union pacoteTipo {
        commandBeginPacket begin;
        commandEndPacket end;
        inventoryResponsePacket inventory;
        tagAccessPacket tagAcess;
        commandWorkPacket work;
    } data;
};


struct fixedQparameters{
    uint8_t qValue;
    uint8_t retryCount;
    bool toggleTarget;
    bool repeatUntilNoTags;
};

struct dinamicQparameters{
    uint8_t startQvalue;
    uint8_t minQvalue;
    uint8_t maxQvalue;
    uint8_t retryCount;
    bool toggleTarget;
    uint8_t thresholdMultiplier;
};

struct singulationAlgorithm {
    uint8_t algorithm; //0=Q fixo, 1=Q dinâmico
    union parameters {
        fixedQparameters fixed;
        dinamicQparameters dinamic;
    } parameters;
};

responsePacket generateResponsePacket(uint8_t bytes[16]);
bool geraPacoteLittleEndian(uint8_t *buffer, commandPacket &command);
bool geraPacoteBigEndian(uint8_t *buffer, commandPacket &command);
bool geraPacoteResponseBigEndian(uint8_t *buffer, responsePacket &response);
uint16_t calculateChecksum(uint8_t *bytesPacote, int numBitsNoPacote);
bool setCheckSum(uint8_t *checkSumDestination, uint8_t *bytesPacote, int numBitsNoPacote);
bool validaChecksum(uint8_t *bytesPacote, int numBitsNoPacote);

bool leRespostaInventario(uint8_t *buffer, int maxBytesNoBuffer, serialSpitFire &serial,
                          std::function<void(reportPacket &)> processamento);

bool enviaComandoSpitFire(spitFire::commandPacket command, uint8_t *pacote, int maxBytesBuffer, serialSpitFire &serial);
bool escrevePacoteSpitFire(uint8_t buffer[16], serialSpitFire &serial);
int lePacoteSpitFire(uint8_t *buffer, int maxBytesBuffer, serialSpitFire &serial);
std::string getStatusMessage(uint8_t stat);
std::string parseEPC(uint8_t *bytesPC_EPC_CRC, int numBytesEPC);

bool configuraAntena(uint8_t antenaLogica, uint8_t portaAntenaFisica, uint16_t potencia, uint16_t timeoutAntena,
                     uint16_t numCiclosInventario, serialSpitFire &serial, bool alteraPotencia, bool alteraTimeout,
                     bool alteraNumCiclos);

void logReportPacket(reportPacket &report, serialSpitFire &serial);

bool printLastMacFirmwareErrorCode(serialSpitFire &serial, uint8_t errorCode[4] = nullptr,
                                   bool clearRegisterAfter = true);

void printResponsePacket(responsePacket &response);

uint32_t getAntennaSenseValue(uint8_t antenaLogica, serialSpitFire &serial);

uint32_t getAntennaSenseThreshold(serialSpitFire &serial);

uint32_t getRFpowerLevels(serialSpitFire &serial);

bool configuraSessao(uint8_t sessao, char target, serialSpitFire &serial);

uint16_t buscaSessaoConfigurada(serialSpitFire &serial);

bool configuraAlgoritmoSingulacaoQdinamico(uint8_t startingQ, uint8_t minQ, uint8_t maxQ, uint8_t retryCount,
                                           bool toggleTarget, uint8_t thresholdMultiplier, serialSpitFire &serial);
bool configuraAlgoritmoSingulacaoQfixo(uint8_t valorQ, uint8_t retryCount, bool toggleTarget,
                                       bool repeatUntilNoTags, serialSpitFire &serial);
bool escolheAlgoritmoDeSingulacao(bool fixedQ, serialSpitFire &serial);

singulationAlgorithm buscaConfiguracoesSingulacao(bool fixedQ, serialSpitFire &serial);

bool escolheLinkProfile(uint8_t profile, serialSpitFire &serial);

}  // namespace spitFire

#endif