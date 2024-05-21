/**
 * @file rc_channels.ino
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief Example of how to read rc channels from a receiver.
 * @version 1.0.2
 * @date 2024-3-19
 *
 * @copyright Copyright (c) 2024, Cassandra "ZZ Cat" Robinson. All rights reserved.
 *
 * @section License GNU General Public License v3.0
 * This example is a part of the CRSF for Arduino library.
 * CRSF for Arduino is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CRSF for Arduino is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with CRSF for Arduino.  If not, see <https://www.gnu.org/licenses/>.
 * 
 */
#include <Arduino.h>
#include "CRSFforArduino.hpp"
#define USE_SERIAL_PLOTTER 0

CRSFforArduino *crsf = nullptr;

int rcChannelCount = crsfProtocol::RC_CHANNEL_COUNT;
const char *rcChannelNames[] = {
    "A",
    "E",
    "T",
    "R",
    "Aux1",
    "Aux2",
    "Aux3",
    "Aux4",

    "Aux5",
    "Aux6",
    "Aux7",
    "Aux8",
    "Aux9",
    "Aux10",
    "Aux11",
    "Aux12"};

void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels);

void setup()
{
    // Initialise the serial port & wait for the port to open.
    Serial.begin(115200);
    while (!Serial)
    {
        ;
    }
        Serial.println("Serial on!");

    // Initialise CRSF for Arduino.
    crsf = new CRSFforArduino(&Serial1, 0, 1);
    /*
     * Skipping Devlopment Board Compatibility Test
     * The SerialReceiver::begin() has beenedited to skip the compatibility test (since it was found to be buggy for some reason)
     */
    if (crsf->begin()){

    }
    else    {
        crsf->end();
        delete crsf;
        crsf = nullptr;
    }

    rcChannelCount = rcChannelCount > crsfProtocol::RC_CHANNEL_COUNT ? crsfProtocol::RC_CHANNEL_COUNT : rcChannelCount;

    crsf->setRcChannelsCallback(onReceiveRcChannels);

    // Show the user that the sketch is ready.
    Serial.println("RC Channels Example");
    delay(1000);
    Serial.println("Ready");
    delay(1000);
}

void loop()
{
    // delay(500);
    crsf->update();
}

void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels)
{
    if (rcChannels->failsafe == false)
    {
        /* Print RC channels every 100 ms. */
        unsigned long thisTime = millis();
        static unsigned long lastTime = millis();

        /* Compensate for millis() overflow. */
        if (thisTime < lastTime)
        {
            lastTime = thisTime;
        }

        if (thisTime - lastTime >= 100)
        {
            lastTime = thisTime;
            for (int i = 1; i <= rcChannelCount; i++)
            {
                Serial.print(rcChannelNames[i - 1]);
                Serial.print(": ");
                Serial.print(crsf->rcToUs(crsf->getChannel(i)));

                if (i < rcChannelCount)
                {
                    Serial.print(", ");
                }
            }
            Serial.println();
        }
    }
}
