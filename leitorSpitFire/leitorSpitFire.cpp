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
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include <functional>
#include <initializer_list>
#include <unordered_map>

#include "serial.h"
#include "spitFire.h"

static serialSpitFire conexaoSerial;
static uint8_t ultimoStatus;
static uint8_t deviceID = 0xFF;  //Broadcasting

//Leitor SpitFire tem portas iniciando em 0.
uint8_t antennasMapping[5] = {0x00, 0x00, 0x01, 0x02, 0x03};

int leitorSpitFire_createReader(const char *uri) {
    conexaoSerial = serialSpitFire(uri);
    conexaoSerial.serialSetup();
    return 1;
}

const char *leitorSpitFire_getStatusMessage() {
    return spitFire::getStatusMessage(ultimoStatus).c_str();
}

void leitorSpitFire_destroyReader() {
    conexaoSerial.closeSerial();
}

int leitorSpitFire_testConnection() {
    int ret = -2;
    spitFire::commandPacket command;
    command.command = 0x01;
    uint8_t pacote[16];
    ;
    if (enviaComandoSpitFire(command, pacote, 200, conexaoSerial)) {
        spitFire::responsePacket response;
        response = spitFire::generateResponsePacket(pacote);
        deviceID = response.returnData[1];
        printf("Device id: %#X\n", deviceID);
        ret = response.returnData[0];
    }
    return ret;
}

static char hexchars[] = "0123456789ABCDEF";
std::string parse(uint8_t *bytes, int numBytesEPC) {
    char string[64];
    char *stringEPC = string;
    while (numBytesEPC--) {
        *stringEPC++ = hexchars[*bytes >> 4];
        *stringEPC++ = hexchars[*bytes & 15];
        bytes++;
    }
    *stringEPC = '\0';
    return std::string(stringEPC);
}

bool leitorSpitFire_alteraConfiguracoes(readerConfig &conf) {
    //Configura sessao
    spitFire::configuraSessao(conf.session, conf.target, conexaoSerial);
    //Configura Algoritmo de Singlacao
    if (conf.dynamicQ) {
        spitFire::configuraAlgoritmoSingulacaoQdinamico(conf.initialQ, 0, 15, 2, false, 7, conexaoSerial);
        spitFire::escolheAlgoritmoDeSingulacao(false, conexaoSerial);
    } else {
        spitFire::configuraAlgoritmoSingulacaoQfixo(conf.Q, 2, false, true, conexaoSerial);
        spitFire::escolheAlgoritmoDeSingulacao(true, conexaoSerial);
    }
    //configura 4 antenas lógicas
    int antenas[4] = {1, 2, 3, 4};
    for (int i = 0; i < 4; i++) {
        bool ok = spitFire::configuraAntena(antenas[i], antennasMapping[antenas[i]], conf.readPower * 10, conf.antennaTimeoutMS,
                                            0, conexaoSerial, true, true, true);
        debug("Configura antena %d: %s", (ok ? "OK":"ERRO"));
    }
    spitFire::escolheLinkProfile(conf.linkProfile,conexaoSerial);
}

bool leitorSpitFire_readTagInventory(tagsReadData *dataContainer, int maxTagsToRead, int readTimeout,
                                     int antenasAler[], int numAntenas, readerConfig &conf) {
    uint8_t pacote[200];
    spitFire::commandPacket command;
    //conexaoSerial.printConnectionStats();
    //Envia comando para trazer DeviceID
    command.command = 0x01;
    enviaComandoSpitFire(command, pacote, 200, conexaoSerial);
    spitFire::responsePacket response;
    response = spitFire::generateResponsePacket(pacote);
    deviceID = response.returnData[1];
    printf("Device id: %#X\n", deviceID);
    //Envia comando para habilitar modo nao continuo
    command.command = 0x02;
    command.deviceID = deviceID;
    command.parameters[0] = 0x01;
    printf("\nEnviando comando Para Habilitar Modo Não Contínuo: ");
    enviaComandoSpitFire(command, pacote, 16, conexaoSerial);
    uint32_t antennaSenseResistanceThreshold = spitFire::getAntennaSenseThreshold(conexaoSerial);
    printf("Limite da Resistência das Antenas: %d ohms?\n", antennaSenseResistanceThreshold);
    bool antenasHabilitadas[5] = {false, false, false, false, false};
    for (int i = 0; i < numAntenas; i++) {
        //spitFire::configuraAntena(antenasAler[i],antenasAler[i],270,0,0,conexaoSerial,true,false,false);
        printf("\n\nEnviando Comando Habilita Antena %d-----------\n", antenasAler[i]);
        command.command = 0x10;
        command.deviceID = deviceID;
        command.parameters[0] = antenasAler[i];
        command.parameters[1] = 0x01;
        enviaComandoSpitFire(command, pacote, 16, conexaoSerial);
        if (antenasAler[i] > 0 && antenasAler[i] < 5) {
            antenasHabilitadas[antenasAler[i]] = true;
        }
        printf("Potencia Conf: %d", conf.readPower);
        //bool ok = spitFire::configuraAntena(antenasAler[i], antennasMapping[antenasAler[i]], conf.readPower * 10, 2000,
        //                                    0, conexaoSerial, true, true, true);
    }
    for (int i = 0; i < 5 && numAntenas < 4; i++) {
        if (!antenasHabilitadas[i]) {
            printf("\n\nEnviando Comando Desabilita Antena %d-----------\n", i);
            command.command = 0x10;
            command.deviceID = deviceID;
            command.parameters[0] = i;
            command.parameters[1] = 0x00;
            enviaComandoSpitFire(command, pacote, 16, conexaoSerial);
        }
    }

    printf("\n\nEnviando Comando Inventário! -----------\n");
    command.command = 0x40;
    command.deviceID = deviceID;
    command.parameters[0] = 0x00;
    command.parameters[1] = 0x00;
    if (!enviaComandoSpitFire(command, pacote, 200, conexaoSerial)) {
        printf("ERRO enviando comando de inventário!");
        spitFire::printLastMacFirmwareErrorCode(conexaoSerial, nullptr, true);
    }

    sleep(1);

    // spitFire::leRespostaInventario(pacote, 200, serial);
    //processaInventarioDeTags(pacote, 200, conexaoSerial, dataContainer);

    std::unordered_map<std::string, int> tagsLidas;
    int contadorTags = 0;
    auto funcaoProcessa = [&tagsLidas, &contadorTags, &dataContainer](spitFire::reportPacket &pacote) {
        //printf("\nProcessa TAG!\n");
        uint16_t tipoPacote = ((uint16_t)pacote.common.reportType[0] << 8) | pacote.common.reportType[1];
        if (tipoPacote == spitFire::INVENTORY_RESPONSE) {
            //uint16_t infoLength = getInfoLength(pacote.common);
            uint16_t protocolControl = ((uint16_t)pacote.data.inventory.inventoryData[1] << 8) | pacote.data.inventory.inventoryData[0];
            int numBytesEPC = protocolControl / 8 * 2;  //8 bits por byte, cada byte são 2 chars
            std::string epcStr = spitFire::parseEPC(pacote.data.inventory.inventoryData, numBytesEPC);
            //std::cout << "EPC:: " << epcStr << std::endl;
            auto pos = tagsLidas.find(epcStr);
            //printf("EPC bytes: %d | HEX: %#04X | contador: %d | ", numBytesEPC, protocolControl, contadorTags);

            if (pos == tagsLidas.end()) {
                //O epc ainda não foi processado. Insere nova Tag no Array e guarda na memoria
                tagsLidas[epcStr] = 1;
                memcpy(dataContainer->tagArray[contadorTags].epc, pacote.data.inventory.inventoryData + 2, numBytesEPC);
                dataContainer->tagArray[contadorTags].rssi = (int32_t)((uint16_t)pacote.data.inventory.rssi[0] << 8) | pacote.data.inventory.rssi[1];
                dataContainer->tagArray[contadorTags].antenna = pacote.data.inventory.logic_ant[1];
                //printf("Antena: %#02X %#02X \n", pacote.data.inventory.logic_ant[0], pacote.data.inventory.logic_ant[1]);
                dataContainer->tagArray[contadorTags].epcByteCount = numBytesEPC;
                contadorTags++;
            } else {
                //Soma o número de vezes em que a tag apareceu no inventario
                tagsLidas[epcStr] = tagsLidas[epcStr] + 1;
                //std::cout << "Tag " << epcStr << ". ReadCount: " << tagsLids[epcStr] << "x!" << std::endl;
                //printf("Antena: %#02X %#02X \n", pacote.data.inventory.logic_ant[0], pacote.data.inventory.logic_ant[1]);
              
            }
        }
    };
    printf("\nMap vazio: %d.\n", tagsLidas.size());
    spitFire::leRespostaInventario(pacote, 200, conexaoSerial, funcaoProcessa);
    printf("\nMap Após Preencher: %d.\n", tagsLidas.size());
    //////////Printing Tags lidas e Contador de vezes
    for(const auto &tag : tagsLidas){
        std::cout << "Tag " << tag.first << ". Lida " << tag.second << "x" << std::endl;
    }
    printf("\n\n\n");

    /* Le A resistência das Antenas após Ativação para DEBUG
    for (int i = 0; i < 5 ; i++) {       
        uint32_t lastMeasuredAntennaSenseResistance = spitFire::getAntennaSenseValue(i,conexaoSerial);
        printf("Ultima medida de Resistência da Antena %d: %d ohms?\n",i,lastMeasuredAntennaSenseResistance);
    }
    //*/
    //Verifica POWER LEVELS para DEBUG
    //spitFire::getRFpowerLevels(conexaoSerial);
    //contadorTags = 0;
    dataContainer->tagCount = contadorTags;
    // contadorTags = 0;
    // auto iterator = tagsLidas.begin();
    // while (iterator != tagsLidas.end()) {
    //     //std::cout << iterator->first << " " << iterator->second << "x" << std::endl;
    //     iterator++;
    //     std::cout << "Cont:" << spitFire::parseEPC(dataContainer->tagArray[contadorTags].epc-2,14) << std::endl;
    //     contadorTags++;
    // }
    printf("\n\n\n''''''''''''''''''''''''''''''''''''\n\n");
    printf("Read %d tags!\n", dataContainer->tagCount);

    return true;
}

/**
 * @param pacote Vetor com 16 posições de uint8_t para guardar os bytes da Mensagem
 * */
bool leitorSpitFire_setReaderSession(uint8_t session, uint8_t pacote[16], serialSpitFire &conexaoSerial) {
    spitFire::commandPacket command;
    command.command = 0x30;
    command.parameters[0] = 0;
    command.parameters[1] = session;
    command.parameters[2] = 0;  //somente A
    return enviaComandoSpitFire(command, pacote, 200, conexaoSerial);
}

/**
 * @param pacote Vetor com 16 posições de uint8_t para guardar os bytes da Mensagem
 * */
bool leitorSpitFire_setSingulationAlgorithm(bool dynamicQ, int Q, int Qinicial, uint8_t pacote[16]) {
    spitFire::commandPacket command;
    command.command = 0x32;
    command.parameters[0] = dynamicQ ? 1 : 0;
    printf("Habilitando Q (0 ? fixo : dinamico): %d\n", dynamicQ);
    bool ok = enviaComandoSpitFire(command, pacote, 200, conexaoSerial);
    //Busca parametros atuais do algoritmo
    command.command = 0x35;
    command.parameters[0] = 0;
    printf("Buscando Parametros Atuais\n");
    ok = enviaComandoSpitFire(command, pacote, 200, conexaoSerial);
    if (ok) {
        spitFire::responsePacket response = spitFire::generateResponsePacket(pacote);
        //Carrega parametros atuais no pacote de envio e sobrescreve só o necessário
        memcpy(command.parameters, response.returnData, 8);
        command.command = 0x34;
        printf("Enviando parametros algoritmo de singulação\n");
        if (dynamicQ) {
            //carregaParametrosQdinamico(command.parameters);
            command.parameters[0] = 1;  //Qdinamico
            command.parameters[1] = Qinicial;
            command.parameters[2] = Qinicial < 2 ? Qinicial : 2;
            command.parameters[3] = 15;
            command.parameters[4] = 2;
            command.parameters[5] = 0;
        } else {
            //carregaParametrosQfixo(command.parameters);
            command.parameters[0] = 0;  //Qfixo
            command.parameters[1] = Q;
            command.parameters[2] = 2;
            command.parameters[3] = 0;
            command.parameters[4] = 0;
            command.parameters[5] = 0;
            command.parameters[6] = 0;
        }
        ok = enviaComandoSpitFire(command, pacote, 200, conexaoSerial);
    }
    return ok;
}

bool leitorSpitfFire_escolheLinkProfile(int tari, int mEncoding, int frequencia, uint8_t pacote[16]) {
    uint8_t linkProfile = 0;
    if (mEncoding == 0) {
        linkProfile = tari < 25 ? 0x03 : 0x00;
    } else {
        linkProfile = frequencia < 300 ? 0x01 : 0x02;
    }
    spitFire::commandPacket command;
    command.command = 0x04;
    command.parameters[0] = linkProfile;
    printf("Enviando pacote LinkProfile: %d", linkProfile);
    return enviaComandoSpitFire(command, pacote, 200, conexaoSerial);
}
