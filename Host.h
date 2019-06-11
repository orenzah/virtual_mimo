//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// Copyright (C) 1992-2015 Andras Varga
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//

#ifndef __ALOHA_HOST_H_
#define __ALOHA_HOST_H_
#define PI 3.14159265359
#include <omnetpp.h>

using namespace omnetpp;

namespace aloha {

/**
 * Aloha host; see NED file for more info.
 */
class Host : public cSimpleModule
{
public:
    double  *distHosts;

  private:
    // routing parameters
    enum { SISO = 0, SIMO = 1, MISO = 2, MIMO = 3 } modeTxRx;
    // parameters
    simtime_t radioDelay;
    int hostId;
    double txRate;
    cPar *iaTime;
    cPar *pkLenBits;
    simtime_t slotTime;
    bool isSlotted;
    cDoubleHistogram totalEnergyStats;


    // state variables, event pointers etc
    //cModule *server;
    cModule **hosts;


    cMessage *endTxEvent;
    enum { IDLE = 0, TRANSMIT = 1 } state;
    simsignal_t stateSignal;
    int pkCounter;

    // position on the canvas, unit is m
    double x, y;

    // speed of light in m/s
    const double propagationSpeed = 299792458.0;

    // animation parameters
    const double ringMaxRadius = 200; // in m
    const double circlesMaxRadius = 100; // in m
    double idleAnimationSpeed;
    double transmissionEdgeAnimationSpeed;
    double midtransmissionAnimationSpeed;

    // figures and animation state
    cPacket *lastPacket = nullptr; // a copy of the last sent message, needed for animation
    mutable cRingFigure *transmissionRing = nullptr; // shows the last packet
    mutable std::vector<cOvalFigure *> transmissionCircles; // ripples inside the packet ring
    //algorithms

    double  *shortestPath;
    double  shortestPathDistance;
    int     *throughPath;
    double *energyHosts;
    bool *neighborSet;
    bool *childrens;
    double totalEnergy = 0;
    double totalEnergyMTD = 0;

    int     myPartnerId = -1;
    int     myParentId = -1;
    int     myParentsPartnerId = -1;  //My parent's partner ID
    double energyAloneToBase;
    double energyPairedToBase;

    /*  region vMER params  */
    double  tp1     = INFINITY;
    double  tp2     = INFINITY;
    int     pnum    = 2;
    bool    rtdTerminated = 0;

    void gotBellmanFord(omnetpp::cMessage* msg);
    void initTxProcess();
    void initBellmanFordProcess();
    void initFamilyProcess();
    void calculateRadioDelay(int i);
    void initDetectionPhase();
    void init_vMER_algo();
    void handleLocationMessage(omnetpp::cMessage* msg);
    void handleBellmanFordMessage(omnetpp::cMessage* msg);
    void sendEnergy(double energy);
    void sendEnergyMTD(double energy);

  public:
    double getEnergyToParentSISO();
    double getEnergyToParentsParentSISO();
    double getEnergyToParentsPartnerSISO();
    double getEnergyToParentsParentsPartnerSISO();
    double getEnergyToPartnerSISO();
    double getEnergyToParentSIMO();
    double getEnergyToParentMISO();
    double getEnergyToParentsPartnerMISO();
    double getEnergyToParentMIMO();


    Host();
    virtual ~Host();

  protected:
    virtual void    initialize() override;
    virtual void    handleMessage(cMessage *msg) override;
    virtual void    refreshDisplay() const override;
    void            sendDCT(int targetHost, bool paired, int hostId);   // detection message
    void            sendPTS(int targetHost);                            // partner-selection message
    void            sendRTD(int targetHost, double pc1, double pc2);    // route-discovery message
    void            setPartner(int targetHost);
    void            setChild(cMessage* msg);
    void            recvDCT(cMessage* msg);
    void            recvPTS(cMessage* msg);
    void            recvRTD(cMessage* msg);
    void            recvEnergy(cMessage* msg);
    void            recvEnergyMTD(cMessage* msg);
    double          getEnergy(int v, int u);

    double          getPath_1_Energy(double pc1); // path1 = u --> v  --> ... --> z
    double          getPath_2_Energy(double pc1); // path2 = u --> w --> v --> ... --> z
    double          getPath_3_Energy(double pc1); // path3 = u --> t  --> ... --> z
    double          getPath_4_Energy(double pc1); // path4 = u --> w  --> t --> ... --> z
    double          getPath_5_Energy(double pc2); // path5 = u --> {v, t} --> ... --> z
    double          getPath_6_Energy(double pc1); // path6 = u --> {u, w} --> v --> ... --> z
    double          getPath_7_Energy(double pc1); // path7 = u --> {u, w} --> t --> ... --> z
    double          getPath_8_Energy(double pc2); // path8 = u --> {u, w} --> {v, t} --> ... --> z

    double calculateEnergyConsumptionPerBit(int _v, int _w, int _t, int numTx, int numRx ,int bitsCount);

    simtime_t getNextTransmissionTime();
};

}; //namespace

#endif

