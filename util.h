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
#ifndef UTIL_H
#define UTIL_H

#include <iostream>
#include <stdlib.h>
#include <sstream>

template<typename F=bool>
void split(int maxTokens, std::string array[], std::string string, char delim, F opt = false){
    if(opt){
        printf("Optional Arg Supplied;");
    }
    std::stringstream stream(string);
    std::string token;
    int index=0;
    while(std::getline(stream, token, delim) && index < maxTokens){
        array[index] = token;
        index++;
    }
    //Preenche o restante do Array com string vazia
    for(index; index < maxTokens; index++){
        array[index] = "";
    }
}

template<typename... Args> 
void debug(char const *printfString, Args... valores){
    #ifdef MAKE_DEBUG
        printf(printfString,valores...);
    #endif
}



#endif