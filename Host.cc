//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// Copyright (C) 1992-2015 Andras Varga
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//

#include <algorithm>

#include "Host.h"
#include <string.h>
using namespace std;
namespace aloha {

Define_Module(Host);

Host::Host()
{
    endTxEvent = nullptr;
}

Host::~Host()
{
    delete lastPacket;
    cancelAndDelete(endTxEvent);
}

void Host::initialize()
{
    stateSignal = registerSignal("state");
    //server = getModuleByPath("server");
    /*if (!server)
        throw cRuntimeError("server not found");
*/
    int numHosts = getParentModule()->par("numHosts");
    hosts = (cModule**)malloc(numHosts * sizeof(cModule));
    distHosts = new double[numHosts];
    neighborSet = new bool[numHosts];
    childrens = new bool[numHosts]();
    energyHosts = new double[numHosts];
    shortestPath = new double[numHosts];
    throughPath = new int[numHosts];
    shortestPathDistance = INFINITY;

    for (int i = 0; i < numHosts; ++i)
    {
        char text[10] = {0};
        shortestPath[i] = INFINITY;
        energyHosts[i]  = INFINITY;


        sprintf(text, "host[%d]", i);
        hosts[i] = getModuleByPath(text);
        if (hosts[i]->getId() == getId())
        {
            hostId = i;
        }
    }

    txRate = par("txRate");
    //iaTime = &par("iaTime");
    pkLenBits = &par("pkLenBits");

    slotTime = par("slotTime");
    isSlotted = slotTime > 0;
    WATCH(slotTime);
    WATCH(isSlotted);
    int *p2 = new int[10]();    // block of ten ints value initialized to 0

    endTxEvent = new cMessage("send/endTx");
    state = IDLE;
    emit(stateSignal, state);
    pkCounter = 0;
    WATCH((int&)state);
    WATCH(pkCounter);

    x = par("x").doubleValue();
    y = par("y").doubleValue();

   // double serverX = server->par("x").doubleValue();
   // double serverY = server->par("y").doubleValue();

    idleAnimationSpeed = par("idleAnimationSpeed");
    transmissionEdgeAnimationSpeed = par("transmissionEdgeAnimationSpeed");
    midtransmissionAnimationSpeed = par("midTransmissionAnimationSpeed");

    //double dist = std::sqrt((x-serverX) * (x-serverX) + (y-serverY) * (y-serverY));
   //radioDelay = dist / propagationSpeed;

    getDisplayString().setTagArg("p", 0, x);
    getDisplayString().setTagArg("p", 1, y);
    if (true || getId() == 3)
    {
        cMessage *msg = new cMessage("initTx");
        scheduleAt(simTime(), msg);
    }

    if (getId() == hosts[0]->getId())
    {
        {
            cMessage *msg = new cMessage("initBellmanFord");
            scheduleAt(simTime() + 0.2, msg);
        }
        {
            cMessage *msg = new cMessage("initDetection");
            scheduleAt(4, msg);
        }
        {
            cMessage *msg = new cMessage("init_vMER");
            scheduleAt(6, msg);
        }
        {
            cMessage *msg = new cMessage("printTotalEnergy");
            scheduleAt(12, msg);
        }

    }

    cMessage *msg = new cMessage("initFamily");
    scheduleAt(3, msg);
    {

        cMessage *msg = new cMessage("initEnergy");
        scheduleAt(7, msg);

    }
    {

        cMessage *msg = new cMessage("initEnergyMTD");
        scheduleAt(10, msg);

    }
    //scheduleAt(getNextTransmissionTime(), endTxEvent);
}

void Host::setChild(cMessage* msg)
{
    const char* messageText = msg->getName();
    int hostNumber = -1;
    //papa-000
    sscanf(messageText, "%*5s%3d", &hostNumber);
    if (hostNumber != -1) {
        childrens[hostNumber] = true;
    }
}

void Host::initTxProcess() {
    int numHosts = getParentModule()->par("numHosts");
    double maxRange = getParentModule()->par("maxRange");
    simtime_t _duration;
    for (int i = 0; i < numHosts; ++i) {
        if (getId() == hosts[i]->getId())
            continue;

        // generate packet and schedule timer when it ends
        double hostX = hosts[i]->par("x").doubleValue();
        double hostY = hosts[i]->par("y").doubleValue();
        double dist = std::sqrt(
                (x - hostX) * (x - hostX) + (y - hostY) * (y - hostY));
        distHosts[i] = dist;
        if (dist > maxRange) {
            distHosts[i] = INFINITY;
            continue;
        }
        radioDelay = dist / propagationSpeed;
        char pkname[40];
        sprintf(pkname, "locationPacket-%03d-#%03d", hostId, pkCounter++);
        EV  << "generating packet " << pkname << endl;
        state = TRANSMIT;
        emit(stateSignal, state);
        cPacket* pk = new cPacket(pkname);
        char parPower[40] = { 0 };
        pk->setBitLength(pkLenBits->intValue());
        simtime_t duration = pk->getBitLength() / txRate;
        _duration = duration;
        pk->setKind(2);
        //pk->setName("loc");
        sendDirect(pk, radioDelay, duration, hosts[i]->gate("in"));
        // let visualization code know about the new packet
        if (transmissionRing != nullptr) {
            delete lastPacket;
            lastPacket = pk->dup();
        }
    }
}

void Host::initBellmanFordProcess() {
    int hostNumber = getParentModule()->par("baseStationId");
    EV  << "Running Bellman-Ford from base station node #" << hostNumber << endl;
    int numHosts = getParentModule()->par("numHosts");
    simtime_t _duration;
    shortestPath[hostId] = 0;
    shortestPathDistance = 0;
    for (int i = 0; i < numHosts; ++i) {
        if (!neighborSet[i])
            continue;

        // generate packet and schedule timer when it ends
        double hostX = hosts[i]->par("x").doubleValue();
        double hostY = hosts[i]->par("y").doubleValue();
        double dist = std::sqrt(
                (x - hostX) * (x - hostX) + (y - hostY) * (y - hostY));
        distHosts[i] = dist;
        radioDelay = dist / propagationSpeed;
        char pkname[60];
        sprintf(pkname, "BellmanFord-%03d-d=%012.3f", 0, 0);
        EV  << "generating packet " << pkname << endl;
        state = TRANSMIT;
        emit(stateSignal, state);
        cPacket* pk = new cPacket(pkname);
        pk->setBitLength(pkLenBits->intValue());
        simtime_t duration = pk->getBitLength() / txRate;
        _duration = duration;
        pk->setKind(3);
        sendDirect(pk, radioDelay, duration, hosts[i]->gate("in"));
        if (transmissionRing != nullptr) {
            delete lastPacket;
            lastPacket = pk->dup();
        }
    }
}

void Host::initFamilyProcess() {
    int numHosts = getParentModule()->par("numHosts");
    double maxRange = getParentModule()->par("maxRange");
    simtime_t _duration;
    int i = -1;
    double min = INFINITY;
    for (int j = 0; j < numHosts; ++j)
    {
        if (shortestPath[j] == shortestPathDistance)
        {
            //min = shortestPath[j];
            i = j;
        }
    }
    //found my papa, now set myParent
    if (i == hostId)
    {
        return;
    }
    if (distHosts[i] > maxRange)
    {
        myParentId = -1;
        return;
    }
    myParentId = i;
    cout << "Distance: " << min << endl;
    cout << "Telling parent I'm its children #" << i << endl;
    // generate packet and schedule timer when it ends
    double hostX = hosts[i]->par("x").doubleValue();
    double hostY = hosts[i]->par("y").doubleValue();
    double dist = std::sqrt((x - hostX) * (x - hostX) + (y - hostY) * (y - hostY));
    distHosts[i] = dist;
    radioDelay = dist / propagationSpeed;
    char pkname[60];
    sprintf(pkname, "papa-%03d", hostId);
    EV  << "generating packet " << pkname << endl;
    state = TRANSMIT;
    emit(stateSignal, state);
    cPacket* pk = new cPacket(pkname);
    pk->setBitLength(pkLenBits->intValue());
    simtime_t duration = pk->getBitLength() / txRate;
    _duration = duration;
    pk->setKind(4);
    sendDirect(pk, radioDelay, duration, hosts[i]->gate("in"));
    if (transmissionRing != nullptr) {
        delete lastPacket;
        lastPacket = pk->dup();
    }
}

void Host::initDetectionPhase()
{
    int numHosts = getParentModule()->par("numHosts");
    for (int i = 0; i < numHosts; ++i)
    {
        if (!childrens[i])
            continue;

        if (i == hostId)
        {
            continue;
        }
        sendDCT(i, 0, 0);
    }
}
void Host::init_vMER_algo()
{
    int numHosts = getParentModule()->par("numHosts");
    for (int i = 0; i < numHosts; ++i)
    {
        if (!childrens[i])
            continue;

        if (i == hostId)
        {
            continue;
        }
        sendRTD(i, 0, INFINITY);
    }
}

void Host::handleLocationMessage(cMessage* msg)
{
    cPacket* pkt = check_and_cast<cPacket*>(msg);
    EV << pkt->getName() << endl;
    int i = 0;
    int numHosts = getParentModule()->par("numHosts");
    //"get sender x y"
    for (int i = 0; i < numHosts; ++i)
    {
        double hostX = hosts[i]->par("x").doubleValue();
        double hostY = hosts[i]->par("y").doubleValue();
        double maxRange = getParentModule()->par("maxRange");
        if (distHosts[i] <= maxRange && getId() != hosts[i]->getId())
        {
            neighborSet[i] = true;
            //energyHosts[i] = 1;
        }
        else
        {
            neighborSet[i] = false;
        }
        for (int i = 0; i < numHosts; ++i)
        {
            EV << "Host " << i << ": distance:" << distHosts[i]  << "   neighbor?  " << neighborSet[i] << "   " << endl;
        }
    }
}

void Host::handleBellmanFordMessage(cMessage* msg)
{
    double newDistance = 0;
    int senderHost = 0;
    const char* messageText = msg->getName();
    sscanf(messageText, "%*18s%lf", &newDistance);
    sscanf(messageText, "%*12s%03d", &senderHost);
    int hostNumber = getParentModule()->par("baseStationId");
    EV << "Running Bellman-Ford from base station node #" << hostNumber << endl;
    int numHosts = getParentModule()->par("numHosts");
    simtime_t _duration;
    double _dist = distHosts[senderHost];
    EV << "My Distance to #" << senderHost << " d=" << _dist << endl;
    EV << "New Distance from #" << senderHost << " d=" << newDistance << endl;
    EV << "My Best d=" << shortestPathDistance << endl;
    if (std::pow(_dist, 2) + newDistance < shortestPathDistance)
    {
        shortestPathDistance = std::pow(_dist, 2) + newDistance;
        shortestPath[senderHost] = shortestPathDistance;
    EV << "Found better path through host[" << senderHost << "] d=" << shortestPathDistance << endl;
    }
    else
    {
        return;
    }
    for (int i = 0; i < numHosts; ++i)
    {
        if (!neighborSet[i])
            continue;

        double dist = distHosts[i];
        radioDelay = dist / propagationSpeed;
        char pkname[60];
        sprintf(pkname, "BellmanFord-%03d-d=%012.5f", hostId, shortestPathDistance);
        EV << "generating packet " << pkname << endl;
        state = TRANSMIT;
        emit(stateSignal, state);
        cPacket* pk = new cPacket(pkname);
        pk->setBitLength(pkLenBits->intValue());
        simtime_t duration = pk->getBitLength() / txRate;
        _duration = duration;
        pk->setKind(3);
        sendDirect(pk, radioDelay, duration, hosts[i]->gate("in"));
        if (transmissionRing != nullptr)
        {
            delete lastPacket;
            lastPacket = pk->dup();
        }
    }
}

void Host::sendEnergy(double energy)
{
    double maxRange = getParentModule()->par("maxRange");
    simtime_t _duration;
    // generate packet and schedule timer when it ends
    double hostX = hosts[this->myParentId]->par("x").doubleValue();
    double hostY = hosts[this->myParentId]->par("y").doubleValue();
    double dist = std::sqrt(
            (x - hostX) * (x - hostX) + (y - hostY) * (y - hostY));
    radioDelay = dist / propagationSpeed;
    char pkname[90];
    sprintf(pkname, "EnergyToRoot-%03d-%042.38lf", this->hostId, energy);
    EV  << "generating packet " << pkname << endl;
    state = TRANSMIT;
    emit(stateSignal, state);
    cPacket* pk = new cPacket(pkname);
    char parPower[40] = { 0 };
    pk->setBitLength(pkLenBits->intValue());
    simtime_t duration = pk->getBitLength() / txRate;
    pk->setKind(8);
    sendDirect(pk, radioDelay, duration, hosts[myParentId]->gate("in"));
    // let visualization code know about the new packet
    if (transmissionRing != nullptr) {
        delete lastPacket;
        lastPacket = pk->dup();
    }
}
void Host::sendEnergyMTD(double energy)
{
    double maxRange = getParentModule()->par("maxRange");
    simtime_t _duration;
    // generate packet and schedule timer when it ends
    double hostX = hosts[this->myParentId]->par("x").doubleValue();
    double hostY = hosts[this->myParentId]->par("y").doubleValue();
    double dist = std::sqrt(
            (x - hostX) * (x - hostX) + (y - hostY) * (y - hostY));
    radioDelay = dist / propagationSpeed;
    char pkname[90];
    sprintf(pkname, "EnergyToRootMTD-%03d-%042.38lf", this->hostId, energy);
    EV  << "generating packet " << pkname << endl;
    state = TRANSMIT;
    emit(stateSignal, state);
    cPacket* pk = new cPacket(pkname);
    char parPower[40] = { 0 };
    pk->setBitLength(pkLenBits->intValue());
    simtime_t duration = pk->getBitLength() / txRate;
    pk->setKind(9);
    sendDirect(pk, radioDelay, duration, hosts[myParentId]->gate("in"));
    // let visualization code know about the new packet
    if (transmissionRing != nullptr) {
        delete lastPacket;
        lastPacket = pk->dup();
    }
}
void Host::handleMessage(cMessage *msg)
{
    //ASSERT(msg == endTxEvent);
    cout << "I'm host[" << hostId << "] " << msg->getName() << endl;
    if (msg->isSelfMessage())
    {
        if (strcmp(msg->getName(), "initTx") == 0)
        {
            initTxProcess();
        }
        else if (strcmp(msg->getName(), "initBellmanFord") == 0)
        {
            initBellmanFordProcess();
        }
        else if (strcmp(msg->getName(), "initFamily") == 0)
        {
            initFamilyProcess();
        }
        else if (strcmp(msg->getName(), "initDetection") == 0)
        {
            initDetectionPhase();
        }
        else if (strcmp(msg->getName(), "init_vMER") == 0)
        {
            init_vMER_algo();
        }
        else if (strcmp(msg->getName(), "initEnergy") == 0)
        {
           if (this->myParentId != -1)
           {
               double energy = std::min(tp1,tp2);
               sendEnergy(energy);
           }
               //cout << "host[" << hostId << "] -> BaseNode: " << temp << endl;
        }
        else if (strcmp(msg->getName(), "initEnergyMTD") == 0)
        {
           if (this->myParentId != -1)
           {
               double energy = getEnergyToParentSISO();
               sendEnergyMTD(energy);
           }
               //cout << "host[" << hostId << "] -> BaseNode: " << temp << endl;
        }
        else if (strcmp(msg->getName(), "printTotalEnergy") == 0)
        {
            cout << totalEnergy << endl;
            cout << totalEnergyMTD << endl;
            simsignal_t energyMTD = getParentModule()->registerSignal("mtd_calc");
            simsignal_t energyMIMO = getParentModule()->registerSignal("mimo_calc");

            emit(energyMTD, totalEnergyMTD);
            emit(energyMIMO, totalEnergy);
               //cout << "host[" << hostId << "] -> BaseNode: " << temp << endl;
        }
    }
    else    //message from the outside
    {
        //if msg is loc
        if(msg->getKind()       == 2)
        {
            handleLocationMessage(msg);
        }
        else if (msg->getKind() == 3)
        {
            handleBellmanFordMessage(msg);
        }
        else if (msg->getKind() == 4)
        {
            setChild(msg);
        }
        else if (msg->getKind() == 5)
        {
            recvDCT(msg);
        }
        else if (msg->getKind() == 6)
        {
            recvPTS(msg);
        }
        else if (msg->getKind() == 7)
        {
            if(rtdTerminated)
                return;
            recvRTD(msg);
        }
        else if (msg->getKind() == 8)
        {
            recvEnergy(msg);
        }
        else if (msg->getKind() == 9)
        {
            recvEnergyMTD(msg);
        }
    }
}

simtime_t Host::getNextTransmissionTime()
{
    simtime_t t = simTime() + iaTime->doubleValue();

    if (!isSlotted)
        return t;
    else
        // align time of next transmission to a slot boundary
        return slotTime * ceil(t/slotTime);
}

void Host::refreshDisplay() const
{
    cCanvas *canvas = getParentModule()->getCanvas();
    const int numCircles = 20;
    const double circleLineWidth = 10;

    // create figures on our first invocation
    if (!transmissionRing) {
        auto color = cFigure::GOOD_DARK_COLORS[getId() % cFigure::NUM_GOOD_DARK_COLORS];

        transmissionRing = new cRingFigure(("Host" + std::to_string(getIndex()) + "Ring").c_str());
        transmissionRing->setOutlined(false);
        transmissionRing->setFillColor(color);
        transmissionRing->setFillOpacity(0.25);
        transmissionRing->setFilled(true);
        transmissionRing->setVisible(false);
        transmissionRing->setZIndex(-1);
        canvas->addFigure(transmissionRing);

        for (int i = 0; i < numCircles; ++i) {
            auto circle = new cOvalFigure(("Host" + std::to_string(getIndex()) + "Circle" + std::to_string(i)).c_str());
            circle->setFilled(false);
            circle->setLineColor(color);
            circle->setLineOpacity(0.75);
            circle->setLineWidth(circleLineWidth);
            circle->setZoomLineWidth(true);
            circle->setVisible(false);
            circle->setZIndex(-0.5);
            transmissionCircles.push_back(circle);
            canvas->addFigure(circle);
        }
    }

    if (lastPacket) {
        // update transmission ring and circles
        if (transmissionRing->getAssociatedObject() != lastPacket) {
            transmissionRing->setAssociatedObject(lastPacket);
            for (auto c : transmissionCircles)
                c->setAssociatedObject(lastPacket);
        }

        simtime_t now = simTime();
        simtime_t frontTravelTime = now - lastPacket->getSendingTime();
        simtime_t backTravelTime = now - (lastPacket->getSendingTime() + lastPacket->getDuration());

        // conversion from time to distance in m using speed
        double frontRadius = std::min(ringMaxRadius, frontTravelTime.dbl() * propagationSpeed);
        double backRadius = backTravelTime.dbl() * propagationSpeed;
        double circleRadiusIncrement = circlesMaxRadius / numCircles;

        // update transmission ring geometry and visibility/opacity
        double opacity = 1.0;
        if (backRadius > ringMaxRadius) {
            transmissionRing->setVisible(false);
            transmissionRing->setAssociatedObject(nullptr);
        }
        else {
            transmissionRing->setVisible(true);
            transmissionRing->setBounds(cFigure::Rectangle(x - frontRadius, y - frontRadius, 2*frontRadius, 2*frontRadius));
            transmissionRing->setInnerRadius(std::max(0.0, std::min(ringMaxRadius, backRadius)));
            if (backRadius > 0)
                opacity = std::max(0.0, 1.0 - backRadius / circlesMaxRadius);
        }

        transmissionRing->setLineOpacity(opacity);
        transmissionRing->setFillOpacity(opacity/5);

        // update transmission circles geometry and visibility/opacity
        double radius0 = std::fmod(frontTravelTime.dbl() * propagationSpeed, circleRadiusIncrement);
        for (int i = 0; i < (int)transmissionCircles.size(); ++i) {
            double circleRadius = std::min(ringMaxRadius, radius0 + i * circleRadiusIncrement);
            if (circleRadius < frontRadius - circleRadiusIncrement/2 && circleRadius > backRadius + circleLineWidth/2) {
                transmissionCircles[i]->setVisible(true);
                transmissionCircles[i]->setBounds(cFigure::Rectangle(x - circleRadius, y - circleRadius, 2*circleRadius, 2*circleRadius));
                transmissionCircles[i]->setLineOpacity(std::max(0.0, 0.2 - 0.2 * (circleRadius / circlesMaxRadius)));
            }
            else
                transmissionCircles[i]->setVisible(false);
        }

        // compute animation speed
        double animSpeed = idleAnimationSpeed;
        if ((frontRadius >= 0 && frontRadius < circlesMaxRadius) || (backRadius >= 0 && backRadius < circlesMaxRadius))
            animSpeed = transmissionEdgeAnimationSpeed;
        if (frontRadius > circlesMaxRadius && backRadius < 0)
            animSpeed = midtransmissionAnimationSpeed;
        canvas->setAnimationSpeed(animSpeed, this);
    }
    else {
        // hide transmission rings, update animation speed
        if (transmissionRing->getAssociatedObject() != nullptr) {
            transmissionRing->setVisible(false);
            transmissionRing->setAssociatedObject(nullptr);

            for (auto c : transmissionCircles) {
                c->setVisible(false);
                c->setAssociatedObject(nullptr);
            }
            canvas->setAnimationSpeed(idleAnimationSpeed, this);
        }
    }

    // update host appearance (color and text)
    getDisplayString().setTagArg("t", 2, "#808000");
    if (state == IDLE) {
        getDisplayString().setTagArg("i", 1, "");
        getDisplayString().setTagArg("t", 0, "");
    }
    else if (state == TRANSMIT) {
        getDisplayString().setTagArg("i", 1, "yellow");
        getDisplayString().setTagArg("t", 0, "TRANSMIT");
    }
}
double Host::calculateEnergyConsumptionPerBit(int _w, int _v,  int _t, int numTx, int numRx ,int bitsCount)
{


    double energyPerBit = 0;
    double constSize = getParentModule()->par("constellation").doubleValue();
    double epsilon = 3*(std::sqrt(std::pow(2,constSize))-1)/(std::sqrt(std::pow(2,constSize))+1);
    double pBitError = getParentModule()->par("bitErrorProbability").doubleValue();
    double gain = std::pow(10,getParentModule()->par("rxtxGain").doubleValue()/10);
    double lambda = getParentModule()->par("waveLength").doubleValue();
    double spectralDensity = std::pow(10,(getParentModule()->par("noiseSpectralDensity").doubleValue()-30)/10);
    double alpha = (epsilon / 0.35) - 1;
    double Ml = std::pow(10,getParentModule()->par("linkMargin").doubleValue()/10);
    double NF = std::pow(10,getParentModule()->par("rxNoiseFigure").doubleValue()/10);

    double Ptc = getParentModule()->par("txConsumption").doubleValue();
    double Psyn = getParentModule()->par("synConsumption").doubleValue();
    double Prc = getParentModule()->par("rxConsumption").doubleValue();
    Ptc /= 1000;
    Psyn /= 1000;
    Prc /= 1000;
    double BW = getParentModule()->par("bandWidth").doubleValue();

    double systemEnergy=0;
    systemEnergy = ((numTx*Ptc)+(2*Psyn)+(numRx*Prc))/(BW*constSize);
    Host *w;
    int _u = this->hostId;
    if (this->myPartnerId != -1)
    {
        w = check_and_cast<Host *>(hosts[this->myPartnerId]);
    }
    Host *v;
    if (_v != -1)
    {
        v = check_and_cast<Host *>(hosts[_v]);
    }

    Host *t = check_and_cast<Host *>(hosts[_t]);
    Host *u = check_and_cast<Host *>(hosts[_u]);

    double dSum = 0;
    if (numTx == 2)
    {
        if (numRx == 2) //MIMO
        {
            //u,w to v,t
            dSum = std::pow(u->distHosts[v->hostId],2) +  std::pow(u->distHosts[t->hostId],2) + std::pow(w->distHosts[v->hostId],2) + std::pow(w->distHosts[t->hostId],2);
            //energyPerBit = ?;
        }
        else    //MISO
        {
            //u,v to t
            //d_ut^2 + d_vt^2
            dSum =  std::pow(u->distHosts[t->hostId],2) + std::pow(v->distHosts[t->hostId],2);
        }
    }
    else
    {
        if (numRx == 2)   //SIMO
        {
            //u to v,t
            //d_uv^2 + d_ut^2
            dSum =  std::pow(u->distHosts[v->hostId],2) + std::pow(u->distHosts[t->hostId],2);
        }
        else    //SISO
        {
            //u to v
            if (_v == -1)
            {
                dSum = INFINITY;
            }
            else
            {
                dSum =  std::pow(u->distHosts[v->hostId],2);
            }
        }
    }
    dSum = (dSum*Ml*NF)/(gain*std::pow(lambda,2));

    energyPerBit = (2.0/3.0)*(1 + alpha) * std::pow(pBitError / 4 , -(1/(numTx*numRx)))*((std::pow(2,constSize) - 1)/(std::pow(constSize, (1/(numTx*numRx+1)))))*spectralDensity*dSum + systemEnergy;

    return energyPerBit;
}
void Host::recvPTS(cMessage* msg)
{
    const char *messageText = msg->getName();
    int numHosts = getParentModule()->par("numHosts");
    //PartnerSelect-000-pts(000,000)
    int senderHost = 0;
    sscanf(messageText, "%*14s%03d", &senderHost);
    for (int i = 0; i < numHosts; ++i)
    {
        double weight = 0;
        if (hostId == 6)
        {
            cout << "host["<< i <<  "] is children: " << !childrens[i] << endl;
            cout << "I am: " << hostId << endl;
        }
        if (!childrens[i]) //if not my child, skip
        {
            continue;
        }
        //else
        if (i == hostId)
        {
            continue;
        }
        sendDCT(i, 1, senderHost);
    }
    setPartner(senderHost);

}
void Host::recvEnergy(cMessage* msg)
{
    const char *messageText = msg->getName();
    int numHosts = getParentModule()->par("numHosts");
    //PartnerSelect-000-pts(000,000)
    int senderHost = 0;
    double energy = 0;

    sscanf(messageText, "%*13s%03d", &senderHost);
    sscanf(messageText, "%*17s%lf", &energy);
    if (this->myParentId != -1)
    {
        double temp = std::min(tp1,tp2);
        if (energy + temp != INFINITY)
            sendEnergy(energy + temp);
    }
    else
    {
        // I'm the base node
        totalEnergy += energy;

        // collect data
    }

}
void Host::recvEnergyMTD(cMessage* msg)
{
    const char *messageText = msg->getName();
    int numHosts = getParentModule()->par("numHosts");
    //PartnerSelect-000-pts(000,000)
    int senderHost = 0;
    double energy = 0;

    sscanf(messageText, "%*16s%03d", &senderHost);
    sscanf(messageText, "%*20s%lf", &energy);
    if (this->myParentId != -1)
    {
        double temp = calculateEnergyConsumptionPerBit(0, this->myParentId, 0, 1, 1, 1);
        if (energy + temp != INFINITY)
            sendEnergyMTD(energy + temp);
    }
    else
    {
        // I'm the base node
        totalEnergyMTD += energy;

        // collect data
    }

}
void Host::recvDCT(cMessage* msg)
{

    const char *messageText = msg->getName();
    int numHosts = getParentModule()->par("numHosts");
    //Detection-%03d-dct(%01d,%03d)
    int senderHost = 0;
    int isPaired = 0;
    int pairedId = 0;
    sscanf(messageText, "%*10s%03d", &senderHost);
    sscanf(messageText, "%*18s%1d", &isPaired);
    sscanf(messageText, "%*20s%03d", &pairedId);

    double gamma = getParentModule()->par("gamma");
    double maximalWeight = -INFINITY;
    int maximallWeightId = -1;
    if (isPaired == 0) //Case (a) papa has a no pair
    {

        for (int i = 0; i < numHosts; ++i)
        {
            double weight = 0;
            if (!childrens[i]) //if not my child, skip
            {
                continue;
            }
            //else
            //calculate the weight W_(u,w) eq. 7 page 6.
            //W_(u,w) = W_(child, self)
            weight = (0.5 - gamma) * calculateEnergyConsumptionPerBit(0, i, 0, 1, 1, 1) - calculateEnergyConsumptionPerBit(i, myParentId, 0, 2, 1, 1);
            //weight = (0.5 - gamma) * getEnergy(i, hostId) + getEnergy(hostId, senderHost)/*TODO -otherThing() */;
            if (weight > maximalWeight)
            {
                maximalWeight = weight;
                maximallWeightId = i;
            }
        }
    }
    else // that is got dct(1,u) a paired message
    {
        myParentsPartnerId = pairedId;
        for (int i = 0; i < numHosts; ++i)
        {
            double weight = 0;
            if (!childrens[i]) //if not my child, skip
            {
                continue;
            }
            //else
            //calculate the weight W_(u,w) eq. 7 page 6.
            //W_(u,w) = W_(child, self)
            double p_uw_v = calculateEnergyConsumptionPerBit(i, myParentId, 0, 2, 1, 1); //p_{u,w}_v
            double p_uw_t = calculateEnergyConsumptionPerBit(i, myParentsPartnerId, 0, 2, 1, 1); //p_{u,w}_t
            double p_uw_vt = calculateEnergyConsumptionPerBit(i, myParentId, myParentsPartnerId, 2, 2, 1); //p_{u,w}_{v,t}
            double temp = std::min(p_uw_v,p_uw_t);
            temp = std::min(temp,p_uw_vt);
            //weight = (0.5 - gamma) * calculateEnergyConsumptionPerBit(0, i, 0, 1, 1, 1);
            weight = (0.5 - gamma) * calculateEnergyConsumptionPerBit(0, i, 0, 1, 1, 1) - temp;
            //weight = (0.5 - gamma) * getEnergy(i, hostId) + getEnergyToParentSISO() - calculateEnergyConsumptionPerBit(i, myParentId, 0, 2, 1, 1)  /*TODO -otherMIMOs() */;
            if (weight > maximalWeight)
            {
                maximalWeight = weight;
                maximallWeightId = i;
            }
        }
    }

    if (maximalWeight > 0)
    {
        int w = maximallWeightId;   // that is the best node to paired with
        int u = hostId;             // that is me, self.
        if (w != hostId)
        {
            sendPTS(w);
            setPartner(w);
        }
        // for-loop send other children dct(1,w), that is I've a partner that is w
        for (int i = 0; i < numHosts; ++i)
        {

            if (!childrens[i] or i == w) //if not my child, skip
            {
                continue;
            }
            if (w == hostId)
            {
                continue;
            }
            //else
            sendDCT(i, 1, w);

        }
    }
    else //no one of children is a good pair, that is not improving the energy costs
    {
        // for-loop send children dct(0.0), that is I don't have a partner
        for (int i = 0; i < numHosts; ++i)
        {

            if (!childrens[i]) //if not my child, skip
            {
                continue;
            }
            if (i == hostId)
            {
                continue;
            }
            //else
            sendDCT(i, 0, 0);

        }
    }

}
void    Host::sendPTS(int targetHost)
{
    double maxRange = getParentModule()->par("maxRange");
    simtime_t _duration;

    // generate packet and schedule timer when it ends
    double hostX = hosts[targetHost]->par("x").doubleValue();
    double hostY = hosts[targetHost]->par("y").doubleValue();
    double dist = std::sqrt((x-hostX) * (x-hostX) + (y-hostY) * (y-hostY));


    radioDelay = dist / propagationSpeed;

    char pkname[40];
    sprintf(pkname, "PartnerSelect-%03d-pts(%03d,%03d)", hostId, targetHost, hostId);

    EV << "generating packet " << pkname << endl;

    state = TRANSMIT;
    emit(stateSignal, state);

    cPacket *pk = new cPacket(pkname);
    char parPower[40] = {0};


    pk->setBitLength(pkLenBits->intValue());
    simtime_t duration = pk->getBitLength() / txRate;
    pk->setKind(6);

    sendDirect(pk, radioDelay, duration, hosts[targetHost]->gate("in"));


    // let visualization code know about the new packet
    if (transmissionRing != nullptr)
    {
        delete lastPacket;
        lastPacket = pk->dup();
    }
}
void    Host::setPartner(int targetHost)
{
    myPartnerId = targetHost;
}
double  Host::getEnergy(int v, int u)
{
    Host *host_u = check_and_cast<Host *>(hosts[u]);
    return host_u->calculateEnergyConsumptionPerBit(0, v, 0, 1, 1, 1);;
}
void Host::sendDCT(int targetHost, bool paired, int hostId)
{
    double maxRange = getParentModule()->par("maxRange");
    simtime_t _duration;

    // generate packet and schedule timer when it ends
    double hostX = hosts[targetHost]->par("x").doubleValue();
    double hostY = hosts[targetHost]->par("y").doubleValue();
    double dist = std::sqrt((x-hostX) * (x-hostX) + (y-hostY) * (y-hostY));


    radioDelay = dist / propagationSpeed;

    char pkname[40];
    sprintf(pkname, "Detection-%03d-dct(%01d,%03d)", this->hostId, paired, hostId);

    EV << "generating packet " << pkname << endl;

    state = TRANSMIT;
    emit(stateSignal, state);

    cPacket *pk = new cPacket(pkname);
    char parPower[40] = {0};


    pk->setBitLength(pkLenBits->intValue());
    simtime_t duration = pk->getBitLength() / txRate;
    pk->setKind(5);

    sendDirect(pk, radioDelay, duration, hosts[targetHost]->gate("in"));


    // let visualization code know about the new packet
    if (transmissionRing != nullptr)
    {
        delete lastPacket;
        lastPacket = pk->dup();
    }
}
void Host::recvRTD(cMessage* msg)
{
    const char *messageText = msg->getName();
    int numHosts = getParentModule()->par("numHosts");
    //RouteDiscovery-000-rtd(000,000000.10000,000000.10000)
    //rtd(%03d,%042.20lf,%042.20lf)
    int senderHost = 0;
    double energyPC1 = 0; //pc1 as refered at vMER algorithm
    double energyPC2 = INFINITY; //pc2 as refered at vMER algorithm
    char checkInf[10] = {0};
    sscanf(messageText, "%*4s%03d", &senderHost);
    sscanf(messageText, "%*8s%042.38lf", &energyPC1);
    sscanf(messageText, "%*90s%3s", &checkInf);
    if (!strcmp(checkInf,"inf"))
    {
        energyPC2 = INFINITY;
    }
    else
    {
        sscanf(messageText, "%*51s%042.38lf", &energyPC2);
    }

    double energy_path0 = INFINITY;
    if (energyPC2 == INFINITY)
    {
        // the parent node v has no partner
        pnum = 0;
        if (myPartnerId == -1)
        {
            tp2 = getEnergyToParentSISO();
            // node u has no partner
            // TODO add this link to route
        }
        else
        {
            double energy_path1 = getPath_1_Energy(energyPC1);
            double energy_path2 = getPath_2_Energy(energyPC1);
            double energy_path3 = getPath_6_Energy(energyPC1);

            // node u has a partner: denoted by w
            // TODO calculate three paths cases
            // TODO select the minimum path energy

            energy_path0 = std::min(energy_path1, energy_path2);
            energy_path0 = std::min(energy_path0, energy_path3);
            tp2 = energyPC1 + getEnergyToParentMISO();
        }
    }
    else
    {
        // the parent node v has partner: denoted {v, t}
        pnum--;
        double maxRange = getParentModule()->par("maxRange");
        Host* v = check_and_cast<Host *>(hosts[this->myParentId]);
        int t = v->myPartnerId;
        if (distHosts[t] > maxRange)
        {
            // node u will not receive message from v
            pnum = 0;
        }
        if (myPartnerId == -1)
        {
            // node u has no partner
            // TODO calculate path1 and path2
            double energy_path1 = getPath_1_Energy(energyPC1);
            double energy_path2 = getPath_5_Energy(energyPC2);
            energy_path0 = std::min(energy_path1, energy_path2);
            // TODO select the minimum path energy as path0
        }
        else
        {
            double energy_path1 = getPath_1_Energy(energyPC1);
            double energy_path2 = getPath_2_Energy(energyPC1);
            double energy_path3 = getPath_6_Energy(energyPC1);
            double energy_path4 = getPath_5_Energy(energyPC2);
            double energy_path5 = getPath_8_Energy(energyPC2);
            // node u has a partner: denoted by w
            // TODO calculate five paths cases
            // TODO select the minimum path energy as path0
            // tp2 = min{energyPC1 + path0, energyPC2 + path0, tp2} TODO

            energy_path0 = std::min(energy_path1,energy_path2);
            energy_path0 = std::min(energy_path0,energy_path3);
            energy_path0 = std::min(energy_path0,energy_path4);
            energy_path0 = std::min(energy_path0,energy_path5);
            double temp = INFINITY;
            temp = std::min(energyPC1 + getEnergyToParentMISO(), energyPC2 + getEnergyToParentMIMO());
            tp2 = std::min(temp, tp2);
        }

    }
    if (pnum == 0)
    {
        // tp1 = energy of path0 TODO
        tp1 = energy_path0;
        // for-loop to send message to connected nodes
        for (int i = 0; i < numHosts; ++i)
        {

            if (!neighborSet[i])
            {
                continue;
            }
            sendRTD(i, tp1,tp2);
            rtdTerminated = 1;
            double tempMin = std::min(tp1,tp2);

        }
    }
}

void Host::sendRTD(int targetHost, double pc1, double pc2)
{
    double maxRange = getParentModule()->par("maxRange");
    simtime_t _duration;

    // generate packet and schedule timer when it ends
    double hostX = hosts[targetHost]->par("x").doubleValue();
    double hostY = hosts[targetHost]->par("y").doubleValue();
    double dist = std::sqrt((x-hostX) * (x-hostX) + (y-hostY) * (y-hostY));


    radioDelay = dist / propagationSpeed;

    char pkname[99];
    double  energyPairedToBase;
    /*
    if (myPartnerId > -1)
    {
        energyPairedToBase = this->energyPairedToBase;
    }
    else
    {
        energyPairedToBase = INFINITY;
    }
    */
    sprintf(pkname, "rtd(%03d,%042.38lf,%042.38lf)", this->hostId, pc1, pc2);


    EV << "generating packet " << pkname << endl;

    state = TRANSMIT;
    emit(stateSignal, state);

    cPacket *pk = new cPacket(pkname);



    pk->setBitLength(pkLenBits->intValue());
    simtime_t duration = pk->getBitLength() / txRate;
    pk->setKind(7);

    sendDirect(pk, radioDelay, duration, hosts[targetHost]->gate("in"));


    // let visualization code know about the new packet
    if (transmissionRing != nullptr)
    {
        delete lastPacket;
        lastPacket = pk->dup();
    }
}

double Host::getPath_1_Energy(double pc1)
{
    if (this->myParentId != -1)
    {
        return pc1 + getEnergyToParentSISO();
    }
    else
    {
        return INFINITY;
    }
}
double Host::getPath_2_Energy(double pc1)
{
    if (this->myPartnerId != -1)
    {
        Host *myPartner = check_and_cast<Host *>(hosts[this->myPartnerId]);
        return pc1 + this->getEnergyToPartnerSISO() + myPartner->getEnergyToParentsParentSISO();
    }
    else
    {
        return INFINITY;
    }
}
double Host::getPath_3_Energy(double pc1)
{
    if (this->myParentsPartnerId != -1 )
    {
        Host *t = check_and_cast<Host *>(hosts[myParentsPartnerId]);
        return pc1 + t->getEnergyToParentSISO() + this->getEnergyToParentsPartnerSISO();
    }
    else
    {
        return INFINITY;
    }
}
double Host::getPath_4_Energy(double pc1)
{
    if (this->myPartnerId != -1 )
    {
        Host *t = check_and_cast<Host *>(hosts[myParentsPartnerId]);
        Host *w = check_and_cast<Host *>(hosts[myPartnerId]);
        return pc1 + t->getEnergyToParentSISO() + this->getEnergyToPartnerSISO() + w->getEnergyToParentsParentsPartnerSISO();
    }
    else
    {
        return INFINITY;
    }
}
double Host::getPath_5_Energy(double pc2)
{
    return this->getEnergyToParentSIMO() + pc2;
}
double Host::getPath_6_Energy(double pc1)
{
    return this->getEnergyToPartnerSISO() + this->getEnergyToParentMISO() + pc1;
}
double Host::getPath_7_Energy(double pc1)
{
    Host *v = check_and_cast<Host *>(hosts[this->myParentId]);
    Host *t = check_and_cast<Host *>(hosts[v->myPartnerId]);
    return this->getEnergyToPartnerSISO() + this->getEnergyToParentsPartnerMISO() + pc1 + t->getEnergyToParentSISO();
}
double Host::getPath_8_Energy(double pc2)
{
    return this->getEnergyToPartnerSISO() + this->getEnergyToParentMIMO() + pc2;
}
double Host::getEnergyToParentSISO()
{
    if (this->myParentId == -1)
    {
       return INFINITY;
    }
    return calculateEnergyConsumptionPerBit(0, this->myParentId, 0, 1, 1, 1);
}
double Host::getEnergyToParentsParentSISO()
{
    Host *v = check_and_cast<Host *>(hosts[this->myParentId]);
    return calculateEnergyConsumptionPerBit(0, v->myParentId, 0, 1, 1, 1);
}
double Host::getEnergyToParentsPartnerSISO()
{
    Host *v = check_and_cast<Host *>(hosts[this->myParentId]);
    if (v->myPartnerId == -1)
        return INFINITY;
    return calculateEnergyConsumptionPerBit(0, v->myPartnerId, 0, 1, 1, 1);
}
double Host::getEnergyToPartnerSISO()
{
    if (this->myPartnerId == -1)
        return INFINITY;
    return calculateEnergyConsumptionPerBit(0, this->myPartnerId, 0, 1, 1, 1);
}
double Host::getEnergyToParentsParentsPartnerSISO()
{
    Host *v = check_and_cast<Host *>(hosts[this->myParentId]);
    Host *_v = check_and_cast<Host *>(hosts[v->myParentId]);
    if (v->myPartnerId == -1)
        return INFINITY;
    return calculateEnergyConsumptionPerBit(_v->myPartnerId, 0, 0, 1, 1, 1);
}
double Host::getEnergyToParentSIMO()
{
    Host *v = check_and_cast<Host *>(hosts[this->myParentId]);
    if (v->myPartnerId == -1)
        return INFINITY;
    return calculateEnergyConsumptionPerBit(0 ,this->myParentId, v->myPartnerId, 1, 2, 1);
}
double Host::getEnergyToParentMISO()
{
    return calculateEnergyConsumptionPerBit(this->myPartnerId, this->myParentId, 0, 2, 1, 1);
}
double Host::getEnergyToParentsPartnerMISO()
{
    Host *v = check_and_cast<Host *>(hosts[this->myParentId]);
    if (v->myPartnerId == -1)
        return INFINITY;
    return calculateEnergyConsumptionPerBit(this->myPartnerId, 0,v->myPartnerId, 2, 1, 1);
}
double Host::getEnergyToParentMIMO()
{
    Host *v = check_and_cast<Host *>(hosts[this->myParentId]);
    if (v->myPartnerId == -1)
        return INFINITY;
    return calculateEnergyConsumptionPerBit(this->myPartnerId, this->myParentId, v->myPartnerId, 2, 2, 1);
}
}; //namespace
