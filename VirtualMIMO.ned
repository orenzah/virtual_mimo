//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// Copyright (C) 1992-2015 Andras Varga
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//


//
// The Aloha network consists of hosts which talk to the central "server" via
// the Aloha or Slotted Aloha protocol
//
network VirtualMIMO
{
    parameters:
        int numHosts;  // number of hosts
        double maxRange @unit(m) = default(200m); //maximum commnunication range        
        double slotTime @unit(ms);  // zero means no slots (pure Aloha)
        
        
        int baseStationId = default(0);
        
        //parameters by Table 1
        double txRate @unit(bps) = default(9600bps);  // transmission rate
        double bandWidth @unit(Hz) = default(10000Hz);
        double constellation @unit(bit) = default(8bit);
        double rxtxGain @unit(dB) = default(5dB);        
        double waveLength @unit(m) = default(0.12m);
        double linkMargin @unit(dB) = default(40dB);
        double rxNoiseFigure @unit(dB) = default(10dB);
        double noiseSpectralDensity @unit(dBmpHz) = default(-171dBmpHz);
        double bitErrorProbability = default(0.001);
        double txConsumption @unit(mW) = default(47.8mW);
        double rxConsumption @unit(mW) = default(69.8mW);
        double synConsumption @unit(mW) = default(50mW);
        double gamma = default(0.1);
        @display("bgi=background/terrain,s;bgb=1000,1000");
    submodules:
        server: Server;
        host[numHosts]: Host {
            txRate = txRate;
            slotTime = slotTime;
        }
}

