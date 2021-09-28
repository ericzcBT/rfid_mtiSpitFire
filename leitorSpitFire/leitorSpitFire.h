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
#ifndef LEITORSPITFIRE_H
#define LEITORSPITFIRE_H

#include "structsTags.h"

int leitorSpitFire_createReader(const char *uri);
const char *leitorSpitFire_getStatusMessage();
void leitorSpitFire_destroyReader();

int leitorSpitFire_testConnection();

bool leitorSpitFire_readTagInventory(tagsReadData *dataContainer, int maxTagsToRead, int readTimeout,
                              int antenasAler[], int numAntenas, readerConfig &conf);

//void processaInventarioDeTags(uint8_t *buffer, int maxBytesNoBuffer, serialSpitFire &serial, tagsReadData *readInfo);
bool leitorSpitFire_alteraConfiguracoes(readerConfig &conf);

#endif