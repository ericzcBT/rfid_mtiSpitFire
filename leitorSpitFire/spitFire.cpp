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
#include "spitFire.h"

#include <bits/stdc++.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include <fstream>
#include <initializer_list>
#include <iostream>
#include <unordered_map>
namespace spitFire {

std::initializer_list<std::pair<uint8_t, int>> initList;

int bytesPaddingUltimoPacote = 0;

uint8_t validHeaderStarts[] = {0x43, 0x52, 0x42, 0x45, 0x49};

uint8_t antenasHabilitadas[4] = {0};

// const uint8_t COMMAND_BEGIN = 0x00;       //0x0000,
// const uint8_t COMMAND_END = 0x01;         //0x0001,
// const uint8_t INVENTORY_RESPONSE = 0x05;  //0x0005,
// const uint8_t TAG_ACESS = 0x06;          //0x0006,
// const uint8_t COMMAND_WORK = 0x0E;        //0x000E
std::unordered_map<int, int> reportSizes({{spitFire::COMMAND_BEGIN, 24},
                                          {spitFire::COMMAND_END, 24},
                                          {spitFire::INVENTORY_RESPONSE, 64},
                                          {spitFire::TAG_ACESS, 64},
                                          {spitFire::COMMAND_WORK, 24}});

/* Processo de Comunicação com o Modulo:
 - Envio de Comando ao Módulo
 - Leitura do Response Packet.
    Status OK
        -Leitura do pacote Command-Begin
        -Leitura do pacote Inventory-Response
        -Leitura dos pacientes TagAccess
        -Leitura do pacote Command-End
    Status ERRO
        - Envia comando para buscar o motivo do ERRO

*/

std::string getStatusMessage(uint8_t stat) {
    switch (stat) {
        case RFID_STATUS_OK:
            return "Status OK!";
        case RFID_ERROR_MODULE_FAILURE:
            return "Module Failure!";
        case RFID_ERROR_INVALID_PARAMETER:
            return "Invalid Parameter!";
        case FALHA_CONTROLADOR:
            return "Falha no Controlador C++!";
        default:
            return "Status Desconhecido!";
    }
    return "Nao pode ocorrer";
}

/*
    Buffer DEVE ser um vetor com tamanho=16, command DEVE ser corretamente inicializado
*/
bool geraPacoteLittleEndian(uint8_t *buffer, commandPacket &command) {
    buffer[0] = command.header[3];
    buffer[1] = command.header[2];
    buffer[2] = command.header[1];
    buffer[3] = command.header[0];
    buffer[4] = command.deviceID;
    buffer[5] = command.command;
    for (int i = 0, j = 6; i < 8; i++, j++) {
        buffer[j] = command.parameters[i];
    }
    spitFire::setCheckSum(command.checksum, buffer, 14 * 8);
    buffer[14] = command.checksum[0];
    buffer[15] = command.checksum[1];
}

/*
    Buffer DEVE ser um vetor com tamanho=16, command DEVE ser corretamente inicializado
*/
bool geraPacoteBigEndian(uint8_t *buffer, commandPacket &command) {
    //memcpy(buffer, command.header, 4);
    buffer[0] = command.header[3];
    buffer[1] = command.header[2];
    buffer[2] = command.header[1];
    buffer[3] = command.header[0];
    buffer[4] = command.deviceID;
    buffer[5] = command.command;
    memcpy(buffer + 6, command.parameters, 8);
    buffer[14] = command.checksum[1];
    buffer[15] = command.checksum[0];
    // printf("\n%#02X ", command.checksum[0]);
    // printf("\n%#02X ", command.checksum[1]);
}
/*
    Buffer DEVE ser um vetor com tamanho=16, command DEVE ser corretamente inicializado
*/
bool geraPacoteResponseBigEndian(uint8_t *buffer, responsePacket &response) {
    //memcpy(buffer, response.header, 4);
    buffer[0] = response.header[3];
    buffer[1] = response.header[2];
    buffer[2] = response.header[1];
    buffer[3] = response.header[0];
    buffer[4] = response.deviceID;
    buffer[5] = response.command;
    memcpy(buffer + 6, response.returnData, 8);
    buffer[14] = response.checksum[0];
    buffer[15] = response.checksum[1];
}

responsePacket generateResponsePacket(uint8_t bytes[16]) {
    responsePacket response;
    int i = 3;
    int j = 0;
    //Mais de um byte é LittleEndian, então inverte a ordem recebida
    for (i = 3, j = 0; i >= 0; i--, j++) {
        response.header[j] = bytes[i];
    }

    response.deviceID = bytes[4];
    response.command = bytes[5];
    for (i = 6, j = 0; i <= 13 && j < 8; i++, j++) {
        response.returnData[j] = bytes[i];
    }
    response.checksum[0] = bytes[15];
    response.checksum[1] = bytes[14];
   //printResponsePacket(response);
    return response;
}

uint16_t calculateChecksum(uint8_t *bytesPacote, int numBitsNoPacote) {
    uint16_t checksum = 0xFFFF;  //Pre-Load 16 bits 1
    uint16_t dataRegister;       //DATA
    uint16_t value;
    for (int i = 0; i < numBitsNoPacote; i++) {
        if ((i % 8) == 0) {                        //Multiplo de 8 bits (0,8,16,...)
            dataRegister = (*bytesPacote++) << 8;  //8 bits do pacote de dados, Dígito mais Significativo primeiro
        }
        value = checksum ^ dataRegister;  //checksum XOR DATA
        //Clocking checksum bit to DataRegister
        checksum = checksum << 1;          //Shift 1 bit para a esquerda
        dataRegister = dataRegister << 1;  //Shift 1 bit para a esquerda
        if (value & 0x8000) {              // 1000000000000000
            checksum = checksum ^ 0x1021;  //checksum XOR polinomio (x^16 + x^12 + x^5 + 1)
        }
    }
    debug("CheckSUM %#02X %#02X", checksum >> 8, checksum & 0xFF);

    return checksum;
}

bool setCheckSum(uint8_t *checkSumDestination, uint8_t *bytesPacote, int numBitsNoPacote) {
    uint16_t checksum = ~calculateChecksum(bytesPacote, numBitsNoPacote);
    checkSumDestination[0] = checksum & 0xFF;  //Bits menos Significativos
    checkSumDestination[1] = checksum >> 8;    //Bits mais Significativos
    return true;
}

bool validaChecksum(uint8_t *bytesPacote, int numBitsNoPacote) {
    uint16_t calculatedChecksum = calculateChecksum(bytesPacote, numBitsNoPacote);  //Calcula o CheckSum incluindo os bits de CHecksum
    //printf("Calculated CheckSum: %" PRIx16 "\n", calculatedChecksum);
    return (calculatedChecksum == 0x1D0F);
}

void encheBufferCommonFields(commonPacketFields &common, uint8_t *buffer) {
    buffer[0] = common.header[3];
    buffer[1] = common.header[2];
    buffer[2] = common.header[1];
    buffer[3] = common.header[0];
    buffer[4] = common.pktRelationNumber;     // = bytes[4];
    buffer[5] = common.pktRelationSequence;   // = bytes[5];
    buffer[6] = common.reportVersion;         // = bytes[6];
    buffer[7] = common.reportFlags;           // = bytes[7];
    buffer[8] = common.reportType[1];         // = bytes[8];
    buffer[9] = common.reportType[0];         // = bytes[9];
    buffer[10] = common.reportInfoLength[0];  // = bytes[10];
    buffer[11] = common.reportInfoLength[1];  // = bytes[11];
    buffer[12] = common.reportSequence[0];    // = bytes[12];
    buffer[13] = common.reportSequence[1];    // = bytes[13];
}

commonPacketFields inicializaCommonFields(uint8_t bytes[14]) {
    commonPacketFields common;
    //Mais de um byte é LittleEndian, então inverte a ordem recebida
    common.header[3] = bytes[0];
    common.header[2] = bytes[1];
    common.header[1] = bytes[2];
    common.header[0] = bytes[3];
    common.pktRelationNumber = bytes[4];
    common.pktRelationSequence = bytes[5];
    common.reportVersion = bytes[6];
    common.reportFlags = bytes[7];
    common.reportType[1] = bytes[8];
    common.reportType[0] = bytes[9];
    common.reportInfoLength[1] = bytes[10];
    common.reportInfoLength[0] = bytes[11];
    common.reportSequence[1] = bytes[12];
    common.reportSequence[0] = bytes[13];

    return common;
}

void encheBufferCommandBegin(commandBeginPacket &begin, uint8_t *bytes) {
    bytes[0] = begin.command[0];
    bytes[1] = begin.command[1];
    bytes[2] = begin.command[2];
    bytes[3] = begin.command[3];
    bytes[4] = begin.msCounter[0];
    bytes[5] = begin.msCounter[1];
    bytes[6] = begin.msCounter[2];
    bytes[7] = begin.msCounter[3];
    bytes[8] = begin.checksum[0];
    bytes[9] = begin.checksum[1];
}

commandBeginPacket geraCommandBegin(uint8_t *bytes, int tamanhoPacote) {
    commandBeginPacket begin;
    begin.command[3] = bytes[0];
    begin.command[2] = bytes[1];
    begin.command[1] = bytes[2];
    begin.command[0] = bytes[3];
    begin.msCounter[3] = bytes[4];
    begin.msCounter[2] = bytes[5];
    begin.msCounter[1] = bytes[6];
    begin.msCounter[0] = bytes[7];
    begin.checksum[0] = bytes[8];
    begin.checksum[1] = bytes[9];
    return begin;
}
commandEndPacket geraCommandEnd(uint8_t *bytes, int tamanhoPacote) {
    commandEndPacket end;
    end.msCounter[3] = bytes[0];
    end.msCounter[2] = bytes[1];
    end.msCounter[1] = bytes[2];
    end.msCounter[0] = bytes[3];
    end.status[3] = bytes[4];
    end.status[2] = bytes[5];
    end.status[1] = bytes[6];
    end.status[0] = bytes[7];
    end.checksum[1] = bytes[8];
    end.checksum[0] = bytes[9];
}

/**
 * O ponteiro deve indicar o primeiro byte do inventário no buffer
 * */
void encheBufferInventory(inventoryResponsePacket &inventory, uint8_t *bytes, uint16_t infoLength) {
    bytes[0] = inventory.counter[0];
    bytes[1] = inventory.counter[1];
    bytes[2] = inventory.counter[2];
    bytes[3] = inventory.counter[3];
    bytes[4] = inventory.nbRSSI;
    bytes[5] = inventory.wbRSSI;
    bytes[6] = inventory.anaControl[0];
    bytes[7] = inventory.anaControl[1];
    bytes[8] = inventory.rssi[0];
    bytes[9] = inventory.rssi[0];
    bytes[10] = inventory.logic_ant[1];
    bytes[11] = inventory.logic_ant[0];
    //Calcula Tamanho do Vetor de dados e Preenche o vetor
    int tamanho = (infoLength - 3) * 4;
    int j = 12;
    //printf("\n--------\nInventoryData %d bytes:", tamanho);
    for (int i = 0; i < tamanho; i++, j++) {
        bytes[j] = inventory.inventoryData[i];
    }
    //printf("\n j: %d", j);
    //Numero de bytes adicionados ao padding(do indice j até o índice 61) - 14 bytes do CommonFields
    tamanho = 61 - 14 - j;
    bytesPaddingUltimoPacote = tamanho;
    debug("\nPadding: %d\n", tamanho);
    inventory.padding = (uint8_t *)malloc(tamanho * sizeof(uint8_t));
    for (int i = 0; i < tamanho; i++, j++) {
        bytes[j] = inventory.padding[i];
    }
    bytes[j] = inventory.checksum[0];
    bytes[j + 1] = inventory.checksum[1];
}

inventoryResponsePacket geraInventoryResponse(uint8_t *bytes, int tamanhoPacote, bool temInfoHardware, uint16_t infoLength) {
    debug("Gerando invResponse\n");
    inventoryResponsePacket inventory;
    inventory.counter[3] = bytes[0];
    inventory.counter[2] = bytes[1];
    inventory.counter[1] = bytes[2];
    inventory.counter[0] = bytes[3];
    inventory.nbRSSI = bytes[4];
    inventory.wbRSSI = bytes[5];

    inventory.anaControl[1] = bytes[6];
    inventory.anaControl[0] = bytes[7];
    inventory.rssi[1] = bytes[8];
    inventory.rssi[0] = bytes[9];
    inventory.logic_ant[1] = bytes[10];
    inventory.logic_ant[0] = bytes[11];
    debug(" antena %#02X %#02X\n", inventory.logic_ant[0], inventory.logic_ant[1]);
    debug("len %#02X", infoLength);
    //Calcula Tamanho do Vetor de dados e Preenche o vetor
    int tamanho = ((infoLength - 3) * 4) - (temInfoHardware ? 8 : 0);
    inventory.inventoryData = (uint8_t *)malloc(tamanho * sizeof(uint8_t));
    int j = 12;
    debug("\n--------\nInventoryData %d bytes:", tamanho);
    for (int i = 0; i < tamanho; i++, j++) {
        inventory.inventoryData[i] = bytes[j];
        //  printf("%#02X ", bytes[j]);
    }
    //printf("\n j: %d", j);
    //Numero de bytes adicionados ao padding(do indice j até o índice 61) - 14 bytes do CommonFields
    tamanho = 61 - 14 - j;
    bytesPaddingUltimoPacote = tamanho;
    //printf("\nPadding: %d\n", tamanho);
    inventory.padding = (uint8_t *)malloc(tamanho * sizeof(uint8_t));
    for (int i = 0; i < tamanho; i++, j++) {
        inventory.padding[i] = bytes[j];
    }
    inventory.checksum[0] = bytes[j];
    inventory.checksum[1] = bytes[j + 1];
    //printf("\nJ: %d\n ", j);
    return inventory;
}
tagAccessPacket geraTagAccess(uint8_t *bytes, int tamanhoPacote) {
    tagAccessPacket access;
    return access;
}
commandWorkPacket geraCommandWork(uint8_t *bytes, int tamanhoPacote) {
    commandWorkPacket work;
    return work;
}

uint16_t getInfoLength(commonPacketFields &common) {
    return ((uint16_t)common.reportInfoLength[0] << 8)  //Digitos mais significativos
           | common.reportInfoLength[1];
}

reportPacket geraReportPacket(uint8_t *bytes, int tamanhoPacote, commonPacketFields common, serialSpitFire &serial) {
    // const uint8_t COMMAND_BEGIN = 0x00;       //0x0000,
    // const uint8_t COMMAND_END = 0x01;         //0x0001,
    // const uint8_t INVENTORY_RESPONSE = 0x05;  //0x0005,
    // const uint8_t TAG_ACESS = 0x06;          //0x0006,
    // const uint8_t COMMAND_WORK = 0x0E;        //0x000E
    bool temInfoHardware = false;
    reportPacket report;
    report.common = common;
    uint16_t infoLength = getInfoLength(common);  //Digitos menos significativos;
    switch (common.reportType[1]) {
        case 0x00:
            debug("\nCommand_Begin\n");
            report.data.begin = geraCommandBegin(bytes, 24);
            break;
        case 0x01:
            debug("\nCommand_End\n");
            report.data.end = geraCommandEnd(bytes, 24);
            break;
        case 0x05:
            debug("\nInventory\n");
            //Pega o valor da flag de Info Hardware (4º bit de um byte)
            temInfoHardware = common.reportFlags & 0b00001000;
            //debug("Flags: %#02X . Tem Infor: %d\n", common.reportFlags, temInfoHardware);
            report.data.inventory = geraInventoryResponse(bytes, 64, temInfoHardware, infoLength);
            break;
        case 0x06:
            debug("\nTag_Access\n");
            report.data.tagAcess = geraTagAccess(bytes, 64);
            break;
        case 0x0E:
            debug("\nCommand_Work\n");
            report.data.work = geraCommandWork(bytes, 24);
            break;
        default:
            printf("Pacote com tipo Desconhecido: %#02X!!\n", common.reportType[1]);
    }
    //#ifdef MAKE_DEBUG
    logReportPacket(report, serial);
    //#endif
    return report;
}
/**
 * @brief Libera a memória Alocada na criação do Pacote. 
 * @param antenaLogica Codigo porta lógica da antena (0 a 4)
 **/
void liberaMemoriaDoPacote(reportPacket &pacote) {
    switch (pacote.common.reportType[1]) {
        case 0x00:
            break;
        case 0x01:
            break;
        case 0x05:
            if (pacote.data.inventory.inventoryData != nullptr) {
                free(pacote.data.inventory.inventoryData);
            }
            if (pacote.data.inventory.padding != nullptr) {
                free(pacote.data.inventory.padding);
            }
            break;
        case 0x06:
            ////////TO-DO função de limpeza da memória alocada com MALLOC para TagAcess
            break;
        case 0x0E:
            break;
        default:
            printf("Pacote com tipo Desconhecido: %#02X!!\n", pacote.common.reportType[1]);
    }
}

int getTamanhoPacotePeloTipo(uint16_t tipo) {
    auto pos = reportSizes.find(tipo);
    if (pos == reportSizes.end()) {  //O tipo informado não existe.
        return -1;
    } else {
        return pos->second;  //Retorna o valor
    }
}

bool escreveComandoSpitFire(uint8_t buffer[16], serialSpitFire &serial) {
    bool escreveuOK = true;
    for (int i = 0; i < 16; i++) {
        escreveuOK &= serial.writeByte(&(buffer[i])) > 0;
    }
    return escreveuOK;
}

bool byteEhInicioDeCabecalho(uint8_t &byte) {
    for (uint8_t cab : validHeaderStarts) {
        if (cab == byte) {
            return true;
        }
    }
    return false;
}

static char hexchars[] = "0123456789ABCDEF";
void TMR_bytesToHex(const uint8_t *bytes, uint32_t size, char *hex) {
    while (size--) {
        *hex++ = hexchars[*bytes >> 4];
        *hex++ = hexchars[*bytes & 15];
        bytes++;
    }
    *hex = '\0';
}

/**
 * @brief Habilita ou Desabilita uma Antena do Sensor. 
 * @param portaAntenaFisica Porta Física em que a Antena foi inserida (1 a 4);
 * @return True se tudo correu bem
 **/
bool alteraStatusAntena(uint8_t portaAntenaFisica, bool ativa, serialSpitFire &serial, uint8_t deviceID) {
    responsePacket response;
    uint8_t stat = 0x01;
    spitFire::commandPacket comando;
    //printf("\n\nEnviando Comando Habilita Antena %d-----------\n", portaAntenaFisica);
    comando.command = 0x10;
    comando.deviceID = deviceID;
    comando.parameters[0] = portaAntenaFisica;
    comando.parameters[1] = ativa ? 0x01 : 0x00;
    uint8_t pacote[16];
    geraPacoteLittleEndian(pacote, comando);
    if (escrevePacoteSpitFire(pacote, serial)) {
        if (lePacoteSpitFire(pacote, 16, serial)) {
            response = generateResponsePacket(pacote);
            stat = response.returnData[0];
        }
    }
    return stat == RFID_STATUS_OK;
}

void printResponsePacket(responsePacket &response) {
    printf("Status: %#X \n", response.returnData[0]);
    printf("Response Data[1...4]: %#X %#X %#X %#X \n", response.returnData[1], response.returnData[2], response.returnData[3], response.returnData[4]);
    printf("Response Data[5...7]: %#X %#X %#X \n", response.returnData[5], response.returnData[6], response.returnData[7]);
}

uint8_t buscaConfiguracoesDaAntena(uint8_t antenaLogica, serialSpitFire &serial, responsePacket &response) {
    uint8_t stat = 10;
    commandPacket comando;
    ;
    comando.command = 0x13;
    comando.parameters[0] = antenaLogica;
    uint8_t pacote[16];
    geraPacoteLittleEndian(pacote, comando);
    //printf("Buscando dados da antena %#02X \n", antenaLogica);
    if (escrevePacoteSpitFire(pacote, serial)) {
        if (lePacoteSpitFire(pacote, 16, serial)) {
            response = generateResponsePacket(pacote);
            stat = response.returnData[0];
            //printf("Porta Física Configurada? %#X: %#X\n", response.returnData[0], response.returnData[7]);
            //printf("Potencia Antena:  %#X: %#X\n", response.returnData[1], response.returnData[2]);
            //printf("Tempo de Ativacao:  %#X: %#X\n", response.returnData[3], response.returnData[4]);
            //printf("Ciclos de Inventario:  %#X: %#X\n", response.returnData[5], response.returnData[6]);
        } else {
            printf("ERRO na LEITURA!\n");
        }
    }
    return stat;
}

/**
 * @brief Configura uma Antena do Sensor. 
 * @param antenaLogica Codigo porta lógica da antena (0 a 4)
 * @param portaAntenaFisica Porta Física em que a Antena foi inserida (1 a 4);
 * @param potencia Potência da antena em décimo de dBm (0 a 270) - 
 * @param timeoutAntena Tempo Máximo em MS de acionamento da antena (0 a 0xFFFF)
 * @param numCiclosInventario Numero máximo de ciclos de inventarios nesta antena (0 a 0xFFFF)
 **/
bool configuraAntena(uint8_t antenaLogica, uint8_t portaAntenaFisica, uint16_t potencia, uint16_t timeoutAntena,
                     uint16_t numCiclosInventario, serialSpitFire &serial, bool alteraPotencia, bool alteraTimeout, bool alteraNumCiclos) {
    uint8_t stat = 0x01;
    //Busca os valores atuais da Antena
    commandPacket comando;
    responsePacket response;
    uint8_t pacote[16];
    stat = buscaConfiguracoesDaAntena(antenaLogica, serial, response);
    if (stat == RFID_STATUS_OK) {
        //std::cout << "Conseguiu os dados da Antena!" << std::endl;
        comando.command = 0x12;
        //std::cout << "Configurando antena:";
        //printf("Fisica %#X, Logica %#X \n", portaAntenaFisica, antenaLogica);
        //Se não deve alterar o parametro, copia do Retorno da mensagem de Busca Parametros
        comando.parameters[0] = antenaLogica;
        if (alteraPotencia) {
            comando.parameters[1] = potencia & 0x00FF;
            comando.parameters[2] = potencia >> 8;
            //printf("Alterando potência: %d = %#X %#X\n", potencia, comando.parameters[1], comando.parameters[2]);
        } else {
            comando.parameters[1] = response.returnData[1];
            comando.parameters[2] = response.returnData[2];
        }
        if (alteraTimeout) {
            comando.parameters[3] = timeoutAntena & 0x00FF;
            comando.parameters[4] = timeoutAntena >> 8;
        } else {
            comando.parameters[3] = response.returnData[3];
            comando.parameters[4] = response.returnData[4];
        }
        if (alteraNumCiclos) {
            comando.parameters[5] = numCiclosInventario & 0x00FF;
            comando.parameters[6] = numCiclosInventario >> 8;
        } else {
            comando.parameters[5] = response.returnData[5];
            comando.parameters[6] = response.returnData[6];
        }
        comando.parameters[7] = portaAntenaFisica;
        geraPacoteLittleEndian(pacote, comando);
        if (escrevePacoteSpitFire(pacote, serial)) {
            if (lePacoteSpitFire(pacote, 16, serial)) {
                response = generateResponsePacket(pacote);
                stat = response.returnData[0];
                //printf("Stat Configura Antena:%#X\n", stat);
            } else {
                stat = 0x01;
                printLastMacFirmwareErrorCode(serial);
            }
        }
        buscaConfiguracoesDaAntena(antenaLogica, serial, response);
    } else {
        //printf("Stat %#X \n", stat);
        std::cout << "ERRO na Operação: " << std::hex << stat << getStatusMessage(stat) << std::endl;
        printLastMacFirmwareErrorCode(serial);
    }
    //std::cout << "Antena Configurada: ";
    //std::cout << getStatusMessage(stat) << std::endl;
    return stat == RFID_STATUS_OK;
}

std::string parseEPC(uint8_t *bytesPC_EPC_CRC, int numBytesEPC) {
    char stringEPC[64];
    //PC configurado em 2 bytes (verificar necessidade de deixar customizável)
    //EPC DEVE ser 16 bytes, precisa confirmar se há como validar o tamanho
    TMR_bytesToHex(bytesPC_EPC_CRC + 2, numBytesEPC, stringEPC);
    return std::string(stringEPC);
}

bool leRespostaInventario(uint8_t *buffer, int maxBytesNoBuffer, serialSpitFire &serial, std::function<void(reportPacket &)> processamento) {
    uint8_t byte;
    uint16_t timer = 0;
    int idx = 0;
    int n = 0;
    int pacotesLidos = 0;
    int readN = 1;
    bool leituraOK = true;
    int tamanhoPacoteAtual = 0;
    int timeout = 10000;
    uint16_t tipoPacote;
    printf("Leitura...\n");
    commonPacketFields common;
    reportPacket report;

    if (!serial.available(0, 1000 * timeout)) {
        printf("Timeout.....");
        return false;
    }
    //*/
    printf(".............................\n");
    n = 0;
    while (leituraOK && n < maxBytesNoBuffer) {
        readN = serial.readByte(&byte);
        leituraOK = readN > 0;
        buffer[n] = byte;
        //printf("%02X ", byte);
        if (n == 0 && !byteEhInicioDeCabecalho(byte) && leituraOK) {
            //printf("Continue...\n");
            continue;
        }
        n += readN;

        if (n == 14) {
            common = inicializaCommonFields(buffer);
            tipoPacote = ((uint16_t)common.reportType[0] << 8);  //Digitos mais significativos
            tipoPacote |= common.reportType[1];
            tamanhoPacoteAtual = getTamanhoPacotePeloTipo(tipoPacote);
            //printf("\nTamanho Pacote: %d  |  Tipo: %#02X %#02X\n", tamanhoPacoteAtual, common.reportType[1], common.reportType[0]);
        }
        if (n == tamanhoPacoteAtual) {
            //Gera um pacote de reporte a partir dos 14 bytes iniciais do common
            //printf("\nGerando pacote com %d bytes!", n);
            report = geraReportPacket(buffer + 14, tamanhoPacoteAtual, common, serial);
            //inverte os bytes do CheckSum para Calcular
            uint8_t temp = buffer[n - 1];
            buffer[n - 1] = buffer[n - 2];
            buffer[n - 2] = temp;
            bool checkSumValido = validaChecksum(buffer, tamanhoPacoteAtual * 8);
            char stringEPC[64];

            if (checkSumValido) {
                //printf("\nCHECKSUM VALIDO!!!\n");
                if (tamanhoPacoteAtual == 64) {
                    TMR_bytesToHex(report.data.inventory.inventoryData, 20, stringEPC);
                    //printf("StringEPC: %s", stringEPC);
                }
                processamento(report);
            } else {
                printf("\nCHECKSUM INVALIDO!!\n");
            }
            liberaMemoriaDoPacote(report);
            tamanhoPacoteAtual = -1;
            n = 0;
            bytesPaddingUltimoPacote = 0;
            pacotesLidos++;
            //printf("...............%d..............\n", pacotesLidos);
#ifdef BUILDING_WSL
            if (common.reportType[1] == COMMAND_END) {
                //Evita problemas ao testar no WSL
                leituraOK = false;
            }
#endif
        }
        if (n == maxBytesNoBuffer - 1) {
            tamanhoPacoteAtual = -1;
            n = 0;
        }
    }
    if (tamanhoPacoteAtual != -1) {
        printf("\n LEITURA FINALIZOU FORA DOS LIMITES DE PACOTES ESPERADOS!! \n");
    }
    return true;
}

bool enviaComandoSpitFire(spitFire::commandPacket command, uint8_t *pacote, int maxBytesBuffer, serialSpitFire &serial) {
    bool sucesso = false;
    spitFire::geraPacoteLittleEndian(pacote, command);
    if (escrevePacoteSpitFire(pacote, serial)) {
        // printf("\nEnviado: ");
        // for (int j = 0; j < 16; j++) {
        //     printf("%#02X ", pacote[j]);
        // }
        //printf("\n");
        if (lePacoteSpitFire(pacote, 16, serial)) {
            // printf("\nLido: ");
            // for (int j = 0; j < 16; j++) {
            //     printf("%#02X ", pacote[j]);
            // }
            // printf("\n");
            printf("Status: %#X", pacote[6]);
            sucesso = true;
        }
    } else {
        printf("Não foi possivel enviar pacote SpitFire!");
    }
    return sucesso;
}

bool escrevePacoteSpitFire(uint8_t buffer[16], serialSpitFire &serial) {
    bool escreveuOK = true;
    for (int i = 0; i < 16; i++) {
        escreveuOK &= serial.writeByte(&(buffer[i])) > 0;
        //printf("%#02X ", buffer[i]);
    }
    return escreveuOK;
}

int lePacoteSpitFire(uint8_t *buffer, int maxBytesBuffer, serialSpitFire &serial) {
    uint8_t byte;
    uint16_t timer = 0;
    int idx = 0;
    int n = 0;
    int readN = 1;
    bool leituraOK = true;
    int timeout = 10000;
    printf("\nLeitura...\n");
    if (!serial.available(0, 1000 * timeout)) {
        printf("Timeout.....");
        return 0;
    }  //*/
    while (leituraOK && n < maxBytesBuffer) {
        readN = serial.readByte(&byte);
        leituraOK = readN > 0;
        if (n < maxBytesBuffer) {
            buffer[n] = byte;
            //printf("%#X ", byte);
        } else {
            printf("\nLendo bytes além do buffer alocado: %#X\n", byte);
        }
        n += readN;
    }
    //printf("\n");
    return n;
}

std::string uintToHexString(uint8_t val) {
    char hexString[3];
    TMR_bytesToHex(&val, 1, hexString);
    std::string result(hexString);
    return result;
}

void logReportPacket(reportPacket &report, serialSpitFire &serial) {
    std::ofstream myfile;
    myfile.open("reportPackets.txt", std::ofstream::app);
    //myfile << "ReportPacket:\n";
    myfile << report.common.header[0] << "|" << report.common.header[1] << "|" << report.common.header[2] << "|" << report.common.header[3] << "|" << std::endl;
    //myfile << uintToHexString(report.common.pktRelationNumber) << "|" << uintToHexString(report.common.pktRelationSequence) << "|" << uintToHexString(report.common.reportVersion) << "|" << uintToHexString(report.common.reportFlags) << std::endl;
    //myfile << uintToHexString(report.common.reportType[0]) << "|" << uintToHexString(report.common.reportType[1]) << "|" << uintToHexString(report.common.reportInfoLength[0]) << "|" << uintToHexString(report.common.reportInfoLength[1]) << "|" << uintToHexString(report.common.reportSequence[0]) << "|" << uintToHexString(report.common.reportSequence[1]) << std::endl;

    switch (report.common.reportType[1]) {
        case 0x00:
            debug("\nCommand_Begin\n");
            myfile << "Command Begin:" << std::endl;
            myfile << "Command:" << uintToHexString(report.data.begin.command[0]) << "|" << uintToHexString(report.data.begin.command[1]) << "|" << uintToHexString(report.data.begin.command[2]) << "|" << uintToHexString(report.data.begin.command[3]) << std::endl;
            myfile << "MSCounter:" << uintToHexString(report.data.begin.msCounter[0]) << "|" << uintToHexString(report.data.begin.msCounter[1]) << "|" << uintToHexString(report.data.begin.msCounter[2]) << "|" << uintToHexString(report.data.begin.msCounter[3]) << std::endl;
            myfile << "Checksum:" << uintToHexString(report.data.begin.checksum[0]) << "|" << uintToHexString(report.data.begin.checksum[1]) << std::endl;
            break;
        case 0x01:
            debug("\nCommand_End\n");
            myfile << "Command End:" << std::endl;
            myfile << "MSCounter:" << uintToHexString(report.data.end.msCounter[0]) << "|" << uintToHexString(report.data.end.msCounter[1]) << "|" << uintToHexString(report.data.end.msCounter[2]) << "|" << uintToHexString(report.data.end.msCounter[3]) << std::endl;
            myfile << "Status:" << uintToHexString(report.data.end.status[0]) << "|" << uintToHexString(report.data.end.status[1]) << "|" << uintToHexString(report.data.end.status[2]) << "|" << uintToHexString(report.data.end.status[3]) << std::endl;
            myfile << "Checksum:" << uintToHexString(report.data.end.checksum[0]) << "|" << uintToHexString(report.data.end.checksum[1]) << std::endl;
            if (report.data.end.status[0] != 0 || report.data.end.status[1] != 0 || report.data.end.status[2] != 0 || report.data.end.status[3] != 0) {
                uint8_t errorCode[4];
                printLastMacFirmwareErrorCode(serial, errorCode);
                myfile << "Last Firmware ErrorCode:" << uintToHexString(errorCode[0]) << "|" << uintToHexString(errorCode[1]);
                myfile << uintToHexString(errorCode[2]) << "|" << uintToHexString(errorCode[3]) << std::endl;
            }
            break;
        case 0x05:
            debug("\nInventory\n");
            myfile << "Inventory:" << std::endl;
            //Pega o valor da flag de Info Hardware (4º bit de um byte)
            //debug("Flags: %#02X . Tem Infor: %d\n", common.reportFlags, temInfoHardware);
            break;
        case 0x06:
            debug("\nTag_Access\n");
            myfile << "Tag Access:" << std::endl;
            break;
        case 0x0E:
            debug("\nCommand_Work\n");
            myfile << "Command Work:" << std::endl;
            break;
    }
    myfile << ".........\n\n";
    myfile.close();
}

void clearLastMacFirmwareErrorCode(serialSpitFire &serial) {
    uint8_t stat = 0x01;
    //Busca os valores atuais da Antena
    commandPacket comando;
    responsePacket response;
    comando.command = 0x62;
    uint8_t pacote[16];
    geraPacoteLittleEndian(pacote, comando);
    printf("Buscando Ultimo erro MACFirmware...\n");
    if (escrevePacoteSpitFire(pacote, serial)) {
        if (lePacoteSpitFire(pacote, 16, serial)) {
            response = generateResponsePacket(pacote);
            stat = response.returnData[0];
            printf("Limpou erro? %d", stat == 0);
            printLastMacFirmwareErrorCode(serial, nullptr, false);
        } else {
            printf("ERRO na LEITURA!\n");
        }
    }
}

/**
 * @brief Printa na tela o Ultimo Codigo de erro do Letiro SpitFire
 **/
bool printLastMacFirmwareErrorCode(serialSpitFire &serial, uint8_t errorCode[4], bool clearRegisterAfter) {
    uint8_t stat = 0x01;
    //Busca os valores atuais da Antena
    commandPacket comando;
    responsePacket response;
    comando.command = 0x63;
    comando.parameters[0] = 0;  //0 = Current Error, 1 = Last Error
    uint8_t pacote[16];
    geraPacoteLittleEndian(pacote, comando);
    printf("Buscando Ultimo erro MACFirmware...\n");
    if (escrevePacoteSpitFire(pacote, serial)) {
        if (lePacoteSpitFire(pacote, 16, serial)) {
            response = generateResponsePacket(pacote);
            stat = response.returnData[0];
        } else {
            printf("ERRO na LEITURA!\n");
        }
    }
    if (stat == RFID_STATUS_OK) {
        std::cout << "Recuperou Código de Erro MacFirmware: ";
        printf("%#X %#X %#X %#X \n", response.returnData[4], response.returnData[3], response.returnData[2], response.returnData[1]);
        if (errorCode != nullptr) {
            errorCode[0] = response.returnData[4];
            errorCode[1] = response.returnData[3];
            errorCode[2] = response.returnData[2];
            errorCode[3] = response.returnData[1];
        }
        if (clearRegisterAfter) {
            // clearLastMacFirmwareErrorCode(serial);
        }
    }
}

uint32_t getAntennaSenseValue(uint8_t antenaLogica, serialSpitFire &serial) {
    uint8_t stat = 0x01;
    //Busca os valores atuais da Antena
    commandPacket comando;
    responsePacket response;
    comando.command = 0x11;
    comando.parameters[0] = antenaLogica;
    uint8_t pacote[16];
    uint32_t senseValue = 0;
    geraPacoteLittleEndian(pacote, comando);
    printf("Buscando Resistencia da Antena:%d...\n", antenaLogica);
    if (escrevePacoteSpitFire(pacote, serial)) {
        if (lePacoteSpitFire(pacote, 16, serial)) {
            response = generateResponsePacket(pacote);
            stat = response.returnData[0];
            if (stat == RFID_STATUS_OK) {
                printf("Antena Habilitada? %#X\n", response.returnData[1]);
                printf("SenseValue: %#X %#X %#X %#X", response.returnData[4], response.returnData[4], response.returnData[3], response.returnData[2]);
                senseValue = response.returnData[5];
                senseValue = (senseValue << 8) + response.returnData[4];
                senseValue = (senseValue << 8) + response.returnData[3];
                senseValue = (senseValue << 8) + response.returnData[2];
                printf(" %d OHMS?\n", senseValue);
            }
        } else {
            printf("ERRO na LEITURA!\n");
        }
    }
    return senseValue;
}

uint32_t getAntennaSenseThreshold(serialSpitFire &serial) {
    uint8_t stat = 0x01;
    //Busca os valores atuais da Antena
    commandPacket comando;
    responsePacket response;
    comando.command = 0x15;
    uint8_t pacote[16];
    uint32_t senseThreshold = 0;
    geraPacoteLittleEndian(pacote, comando);
    printf("Buscando Limite de Resistencia das Antenas...\n");
    if (escrevePacoteSpitFire(pacote, serial)) {
        if (lePacoteSpitFire(pacote, 16, serial)) {
            response = generateResponsePacket(pacote);
            stat = response.returnData[0];
            if (stat == RFID_STATUS_OK) {
                printf("senseThreshold: %#X %#X %#X %#X", response.returnData[4], response.returnData[3], response.returnData[2], response.returnData[1]);
                senseThreshold = response.returnData[4];
                senseThreshold = (senseThreshold << 8) + response.returnData[3];
                senseThreshold = (senseThreshold << 8) + response.returnData[2];
                senseThreshold = (senseThreshold << 8) + response.returnData[1];
                printf(" %d OHMS?\n", senseThreshold);
            }
        } else {
            printf("ERRO na LEITURA!\n");
        }
    }
    return senseThreshold;
}

double twosComplementToLong(uint16_t original) {
    double converted = original / 256;
    printf("Complemento de 2 convertido: %lf 0.1ohms, %lf ohms", converted, converted * 0.1);
}

uint32_t getRFpowerLevels(serialSpitFire &serial) {
    uint8_t stat = 0x01;
    //Busca os valores atuais da Antena
    commandPacket comando;
    responsePacket response;
    comando.command = 0x93;
    comando.parameters[0] = 0;  //Forward RF power
    uint8_t pacote[16];
    uint32_t powerLevels = 0;
    uint16_t forwardRFpowerLevel = 0;
    uint16_t reverseRFpowerLevel = 0;
    geraPacoteLittleEndian(pacote, comando);
    printf("Buscando RF PowerLevels...\n");
    if (escrevePacoteSpitFire(pacote, serial)) {
        if (lePacoteSpitFire(pacote, 16, serial)) {
            response = generateResponsePacket(pacote);
            stat = response.returnData[0];
            if (stat == RFID_STATUS_OK) {
                printf("forward RF PowerLevel: %#X %#X = ", response.returnData[2], response.returnData[1]);
                forwardRFpowerLevel = response.returnData[2];
                forwardRFpowerLevel = (forwardRFpowerLevel << 8) + response.returnData[1];
                printf(" %d 0.1dBm | %lf dBm\n", forwardRFpowerLevel, ((double)forwardRFpowerLevel) * 0.1);
                twosComplementToLong(forwardRFpowerLevel);
            }
        } else {
            printf("ERRO na LEITURA!\n");
        }
    }
    comando.parameters[0] = 1;  //Reverse RF powerLevel
    geraPacoteLittleEndian(pacote, comando);
    if (escrevePacoteSpitFire(pacote, serial)) {
        if (lePacoteSpitFire(pacote, 16, serial)) {
            response = generateResponsePacket(pacote);
            stat = response.returnData[0];
            if (stat == RFID_STATUS_OK) {
                printf("reverse RF PowerLevel: %#X %#X = ", response.returnData[2], response.returnData[1]);
                reverseRFpowerLevel = response.returnData[2];
                reverseRFpowerLevel = (reverseRFpowerLevel << 8) + response.returnData[1];
                printf(" %d 0.1dBm | %lf dBm\n", reverseRFpowerLevel, ((double)reverseRFpowerLevel) * 0.1);
                twosComplementToLong(reverseRFpowerLevel);
            }
        } else {
            printf("ERRO na LEITURA!\n");
        }
    }
    powerLevels = forwardRFpowerLevel;
    powerLevels = (powerLevels << 16) + reverseRFpowerLevel;
    return powerLevels;
}

bool configuraSessao(uint8_t sessao, char target, serialSpitFire &serial) {
    uint8_t stat = 0x01;
    bool configurou = false;
    bool parametrosOK = true;
    if (sessao > 3) {
        printf("Valor da Sessão deve ser especificado entre 0 e 3\n");
        parametrosOK = false;
    }
    if (target != 'a' && target != 'A' && target != 'b' && target != 'B') {
        printf("Alvo da sessão deve ser A ou B!\n");
        parametrosOK = false;
    }
    if (parametrosOK) {
        uint16_t valoresAtuais = buscaSessaoConfigurada(serial);
        uint8_t sessaoAtual = valoresAtuais >> 8;
        uint8_t targetAtual = valoresAtuais;
        printf("\nSessaoTarget: %#X, Sessao %#X, Target %#X\n", valoresAtuais,sessaoAtual,targetAtual);
        uint8_t newTarget = (target == 'a' || target == 'A') ? 0 : 1;
        //Valores já estão configurados, não precisa fazer nada.
        if (sessaoAtual == sessao && targetAtual == newTarget) {
            configurou = true;
        } else {
            commandPacket comando;
            responsePacket response;
            comando.command = 0x30;
            comando.parameters[0] = 0;          //0=All selected Flag, 2=Deasserted Select, 3=Asserted Select
            comando.parameters[1] = sessao;     //S0,S1,S2 ou S3
            comando.parameters[2] = newTarget;  //Forward RF power
            uint8_t pacote[16];
            geraPacoteLittleEndian(pacote, comando);
            printf("Configurando Sessao %d, target %c...\n", sessao, target);
            if (escrevePacoteSpitFire(pacote, serial)) {
                if (lePacoteSpitFire(pacote, 16, serial)) {
                    response = generateResponsePacket(pacote);
                    stat = response.returnData[0];
                    if (stat == RFID_STATUS_OK) {
                        configurou = true;
                        printf("Config OK!\n");
                    }else{
                        printf("ERRO na configuração: %#X\n", stat);
                    }
                } else {
                    printf("ERRO na LEITURA!\n");
                }
            }
        }
    }
    return configurou;
}

uint16_t buscaSessaoConfigurada(serialSpitFire &serial) {
    uint8_t stat = 0x01;
    uint16_t sessaoEtarget = 0;
    bool configurou = false;
    commandPacket comando;
    responsePacket response;
    comando.command = 0x31;
    uint8_t pacote[16];
    geraPacoteLittleEndian(pacote, comando);
    if (escrevePacoteSpitFire(pacote, serial)) {
        if (lePacoteSpitFire(pacote, 16, serial)) {
            response = generateResponsePacket(pacote);
            stat = response.returnData[0];
            if (stat == RFID_STATUS_OK) {
                printf("Sessao %d, target %c\n", response.returnData[2], response.returnData[3] == 0 ? 'A' : 'B');
                sessaoEtarget = response.returnData[2];
                sessaoEtarget = (sessaoEtarget << 8) + response.returnData[3];
            }
        } else {
            printf("ERRO na LEITURA!\n");
        }
    }

    return sessaoEtarget;
}

bool configuraAlgoritmoDeSingulacao(commandPacket comandoConfiguracoes, serialSpitFire &serial) {
    uint8_t stat = 0x01;
    bool configurou = false;
    responsePacket response;
    uint8_t pacote[16];
    //printf("Enviando comando Configuracao!\n");
    geraPacoteLittleEndian(pacote, comandoConfiguracoes);
    if (escrevePacoteSpitFire(pacote, serial)) {
        if (lePacoteSpitFire(pacote, 16, serial)) {
            response = generateResponsePacket(pacote);
            stat = response.returnData[0];
            configurou = true;
        } else {
            printf("ERRO na LEITURA!\n");
        }
    }

    return configurou;
}  // namespace spitFire

void printConfiguracoesSingulacao(singulationAlgorithm confs) {
    if (confs.algorithm == 0) {
        printf("Algoritmo Q Fixo\n");
        printf("Q: %d\n", confs.parameters.fixed.qValue);
        printf("RetryCount: %d\n", confs.parameters.fixed.retryCount);
        printf("Toggle Target: %d\n", confs.parameters.fixed.toggleTarget);
        printf("Repeat Until No Tags: %d\n", confs.parameters.fixed.repeatUntilNoTags);
    } else {
        printf("Algoritmo Q Dinamico\n");
        printf("Starting Q: %d\n", confs.parameters.dinamic.startQvalue);
        printf("Min Q: %d\n", confs.parameters.dinamic.minQvalue);
        printf("Max Q: %d\n", confs.parameters.dinamic.maxQvalue);
        printf("Retry Count: %d\n", confs.parameters.dinamic.retryCount);
        printf("Toggle Target: %d\n", confs.parameters.dinamic.toggleTarget);
        printf("Threshold Multiplier: %d\n", confs.parameters.dinamic.thresholdMultiplier);
    }
}

bool configuraAlgoritmoSingulacaoQdinamico(uint8_t startingQ, uint8_t minQ, uint8_t maxQ, uint8_t retryCount,
                                           bool toggleTarget, uint8_t thresholdMultiplier, serialSpitFire &serial) {
    if (maxQ > 15 || minQ > 15 || startingQ > 15) {
        printf("Valores de Q devem ser entre 0 e 15!\n");
    }
    if (startingQ < minQ || maxQ < minQ || maxQ < startingQ) {
        printf("Valores de Q devem seguir a ordem:\nminQ < startingQ < maxQ!\n");
    }
    
    commandPacket comandoConfigs;
    comandoConfigs.command = 0x34;
    comandoConfigs.parameters[0] = 1;
    comandoConfigs.parameters[1] = startingQ;
    comandoConfigs.parameters[2] = minQ;
    comandoConfigs.parameters[3] = maxQ;
    comandoConfigs.parameters[4] = retryCount;
    comandoConfigs.parameters[5] = toggleTarget ? 1 : 0;
    comandoConfigs.parameters[6] = thresholdMultiplier;
    configuraAlgoritmoDeSingulacao(comandoConfigs, serial);
    printf("Configurou algoritmo de Singulação: \n");
    singulationAlgorithm confsAtuais = buscaConfiguracoesSingulacao(false, serial);
    printConfiguracoesSingulacao(confsAtuais);
}

bool configuraAlgoritmoSingulacaoQfixo(uint8_t valorQ, uint8_t retryCount, bool toggleTarget,
                                       bool repeatUntilNoTags, serialSpitFire &serial) {
    if (valorQ > 15) {
        printf("Valor de Q deve ser entre 0 e 15!\n");
    }

    commandPacket comandoConfigs;
    comandoConfigs.command = 0x34;
    comandoConfigs.parameters[0] = 0;
    comandoConfigs.parameters[1] = valorQ;
    comandoConfigs.parameters[2] = retryCount;
    comandoConfigs.parameters[3] = toggleTarget ? 1 : 0;
    comandoConfigs.parameters[4] = repeatUntilNoTags ? 1 : 0;
    comandoConfigs.parameters[5] = 0;
    comandoConfigs.parameters[6] = 0;
    configuraAlgoritmoDeSingulacao(comandoConfigs, serial);
    printf("Configurou algoritmo de Singulação: \n");
    singulationAlgorithm confsAtuais = buscaConfiguracoesSingulacao(true, serial);
    printConfiguracoesSingulacao(confsAtuais);
}

bool escolheAlgoritmoDeSingulacao(bool fixedQ, serialSpitFire &serial) {
    uint8_t stat = 0x01;
    bool configurou = false;
    commandPacket comando;
    responsePacket response;
    comando.command = 0x32;
    comando.parameters[0] = fixedQ ? 0 : 1;
    uint8_t pacote[16];
    geraPacoteLittleEndian(pacote, comando);
    printf("Configurando Algoritmo para Q %s\n", fixedQ ? "fixo" : "dinamico");
    if (escrevePacoteSpitFire(pacote, serial)) {
        if (lePacoteSpitFire(pacote, 16, serial)) {
            response = generateResponsePacket(pacote);
            stat = response.returnData[0];
            if (stat == RFID_STATUS_OK) {
                configurou = true;
            } else {
                printf("ERRO na LEITURA!\n");
            }
        }
    }
    return configurou;
}

singulationAlgorithm buscaConfiguracoesSingulacao(bool fixedQ, serialSpitFire &serial) {
    singulationAlgorithm confs;
    uint8_t stat = 0x01;
    bool configurou = false;
    commandPacket comando;
    responsePacket response;
    comando.command = 0x35;
    comando.parameters[0] = fixedQ ? 0 : 1;
    uint8_t pacote[16];
    printf("Enviando comando Configuracao!\n");
    geraPacoteLittleEndian(pacote, comando);
    if (escrevePacoteSpitFire(pacote, serial)) {
        if (lePacoteSpitFire(pacote, 16, serial)) {
            response = generateResponsePacket(pacote);
            stat = response.returnData[0];
            if (stat == RFID_STATUS_OK) {
                if (fixedQ) {
                    confs.algorithm = 0;
                    fixedQparameters params;
                    params.qValue = response.returnData[1];
                    params.retryCount = response.returnData[2];
                    params.toggleTarget = response.returnData[3];
                    params.repeatUntilNoTags = response.returnData[4];
                    confs.parameters.fixed = params;
                } else {
                    confs.algorithm = 1;
                    dinamicQparameters params;
                    params.startQvalue = response.returnData[1];
                    params.minQvalue = response.returnData[2];
                    params.maxQvalue = response.returnData[3];
                    params.retryCount = response.returnData[4];
                    params.toggleTarget = response.returnData[5];
                    params.thresholdMultiplier = response.returnData[6];
                    confs.parameters.dinamic = params;
                }
            }
        } else {
            printf("ERRO na LEITURA!\n");
        }
    }
    return confs;
}

bool escolheLinkProfile(uint8_t profile, serialSpitFire &serial) {
    uint8_t stat = 0x01;
    bool configurou = false;
    commandPacket comando;
    responsePacket response;
    comando.command = 0x04;
    comando.parameters[0] = profile;
    uint8_t pacote[16];
    geraPacoteLittleEndian(pacote, comando);
    printf("Alterando LinkProfile para %d\n", profile);
    if (escrevePacoteSpitFire(pacote, serial)) {
        if (lePacoteSpitFire(pacote, 16, serial)) {
            response = generateResponsePacket(pacote);
            stat = response.returnData[0];
            if (stat == RFID_STATUS_OK) {
                configurou = true;
                printf("Link Profile alterado!\n");
            } else {
                printf("ERRO na LEITURA!\n");
            }
        }
    }
    return configurou;
}


}  // namespace spitFire