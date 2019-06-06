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
#define INFINITY (1e999)
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
    server = getModuleByPath("server");
    if (!server)
        throw cRuntimeError("server not found");

    int numHosts = getParentModule()->par("numHosts");
    hosts = (cModule**)malloc(numHosts * sizeof(cModule));
    distHosts = new double[numHosts];
    neighborSet = new bool[numHosts];
    energyHosts = new double[numHosts];
    shortestPath = new double[numHosts];
    throughPath = new int[numHosts];
    for (int i = 0; i < numHosts; ++i)
    {
        char text[10] = {0};
        shortestPath[i] = INFINITY;
        sprintf(text, "host[%d]", i);
        hosts[i] = getModuleByPath(text);
    }

    txRate = par("txRate");
    iaTime = &par("iaTime");
    pkLenBits = &par("pkLenBits");

    slotTime = par("slotTime");
    isSlotted = slotTime > 0;
    WATCH(slotTime);
    WATCH(isSlotted);

    endTxEvent = new cMessage("send/endTx");
    state = IDLE;
    emit(stateSignal, state);
    pkCounter = 0;
    WATCH((int&)state);
    WATCH(pkCounter);

    x = par("x").doubleValue();
    y = par("y").doubleValue();

    double serverX = server->par("x").doubleValue();
    double serverY = server->par("y").doubleValue();

    idleAnimationSpeed = par("idleAnimationSpeed");
    transmissionEdgeAnimationSpeed = par("transmissionEdgeAnimationSpeed");
    midtransmissionAnimationSpeed = par("midTransmissionAnimationSpeed");

    double dist = std::sqrt((x-serverX) * (x-serverX) + (y-serverY) * (y-serverY));
    radioDelay = dist / propagationSpeed;

    getDisplayString().setTagArg("p", 0, x);
    getDisplayString().setTagArg("p", 1, y);
    if (true || getId() == 3)
    {
        cMessage *msg = new cMessage("initTx");
        scheduleAt(simTime(), msg);
    }

    if (getId() == hosts[0]->getId())
    {
        cMessage *msg = new cMessage("initBellmanFord");
        scheduleAt(simTime() + 0.2, msg);
    }

    //scheduleAt(getNextTransmissionTime(), endTxEvent);
}

void Host::handleMessage(cMessage *msg)
{
    //ASSERT(msg == endTxEvent);
    if (msg->isSelfMessage())
    {
        if (strcmp(msg->getName(), "initTx") == 0)
        {

            int numHosts = getParentModule()->par("numHosts");
            simtime_t _duration;
            for (int i = 0; i < numHosts; ++i)
            {
                if (getId() == hosts[i]->getId())
                    continue;
                // generate packet and schedule timer when it ends
                double hostX = hosts[i]->par("x").doubleValue();
                double hostY = hosts[i]->par("y").doubleValue();
                double dist = std::sqrt((x-hostX) * (x-hostX) + (y-hostY) * (y-hostY));
                distHosts[i] = dist;
                radioDelay = dist / propagationSpeed;

                char pkname[40];
                sprintf(pkname, "locationPacket-%03d-#%03d", getId(), pkCounter++);

                EV << "generating packet " << pkname << endl;

                state = TRANSMIT;
                emit(stateSignal, state);

                cPacket *pk = new cPacket(pkname);
                char parPower[40] = {0};


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
        else if (strcmp(msg->getName(), "initBellmanFord") == 0)
        {
            int hostNumber = getParentModule()->par("baseStationId");
            EV << "Running Bellman-Ford from base station node #" << hostNumber << endl;
            int numHosts = getParentModule()->par("numHosts");
            simtime_t _duration;
            for (int i = 0; i < numHosts; ++i)
            {
                 if (!neighborSet[i])
                     continue;
                 // generate packet and schedule timer when it ends
                 double hostX = hosts[i]->par("x").doubleValue();
                 double hostY = hosts[i]->par("y").doubleValue();
                 double dist = std::sqrt((x-hostX) * (x-hostX) + (y-hostY) * (y-hostY));
                 distHosts[i] = dist;
                 radioDelay = dist / propagationSpeed;

                 char pkname[60];
                 sprintf(pkname, "bellmanFord-%03d-d=%012.3f", i, 0);

                 EV << "generating packet " << pkname << endl;

                 state = TRANSMIT;
                 emit(stateSignal, state);

                 cPacket *pk = new cPacket(pkname);



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
    }
    else    //message from the outside
    {
        //if msg is loc
        if(msg->getKind()==2)
        {


            cPacket *pkt = check_and_cast<cPacket *>(msg);
            EV << pkt->getName() << endl;
            int i=0;

            int numHosts = getParentModule()->par("numHosts");
            //"get sender x y"
            for (int i = 0; i < numHosts; ++i)
            {
                double hostX = hosts[i]->par("x").doubleValue();
                double hostY = hosts[i]->par("y").doubleValue();
                double maxRange = getParentModule()->par("maxRange");
                if (distHosts[i] <= maxRange and getId() != hosts[i]->getId())
                {
                    neighborSet[i]=true;
                    energyHosts[i]= calculateEnergeyConsumptionPerBit(hostX,hostY,0,0,pkt->getBitLength());
                    //energyHosts[i] = 1;
                }
                else
                {
                    neighborSet[i]=false;
                    energyHosts[i]=INFINITY;
                }
                for (int i = 0; i < numHosts; ++i)
                {
                    EV<<"Host "<<i<<": distance:"<< distHosts[i]<< "   energy:"<< energyHosts[i]<<"   neighbor?  "<<neighborSet[i]<<"   "<<endl;
                }

            }
        }
        else if (msg->getKind() == 3)
        {
            double newDistance = 0;
            int senderHost = 0;
            const char *messageText = msg->getName();
            sscanf(messageText, "%*18s%012.3f", &newDistance);
            sscanf(messageText, "%*12s%03d", &senderHost);
            int hostNumber = getParentModule()->par("baseStationId");
            EV << "Running Bellman-Ford from base station node #" << hostNumber << endl;
            int numHosts = getParentModule()->par("numHosts");
            simtime_t _duration;
            for (int i = 0; i < numHosts; ++i)
            {
                 if (!neighborSet[i])
                     continue;

                 double dist = distHosts[i];
                 radioDelay = dist / propagationSpeed;
                 if (std::pow(dist,2)  + newDistance < shortestPath[i])
                 {
                     shortestPath[i] = std::pow(dist,2) + newDistance;
                     //TODO
                     //fix the distance, should be powered by 2
                     throughPath[i] = senderHost;
                     EV << "Found better path through host[" << senderHost  << "]" << endl;
                 }
                 else
                 {
                     continue;
                 }

                 char pkname[60];
                 sprintf(pkname, "bellmanFord-%03d-d=%012.3f", i, shortestPath[i]);

                 EV << "generating packet " << pkname << endl;

                 state = TRANSMIT;
                 emit(stateSignal, state);

                 cPacket *pk = new cPacket(pkname);



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
double Host::calculateEnergeyConsumptionPerBit(double x, double y,int numTx, int numRx ,int bitsCount)
{
    //TODO
    /*
    double energyPerBit = 0;
    double constSize = getParentModule()->par("constellation").doubleValue();
    double epsilon = 3*(std::sqrt(std::pow(2,constSize))-1)/(std::sqrt(std::pow(2,constSize))+1);
    double pBitError = getParentModule()->par("bitErrorProbability").doubleValue();

    double spectralDensity = getParentModule()->par("noiseSpectralDensity").doubleValue();
    double alpha = (epsilon / 0.35) - 1;

    energyPerBit = (2.0/3.0)*(1 + alpha) * std::pow(pBitError / 4 , -(1/(numTx*numRx)))*((std::pow(2,constSize) - 1)/(std::pow(constSize, (1/(numTx*numRx+1)))))*spectralDensity;
    double dSum = 0;
    for (int i = 1;i < numTx; ++i)
    {
        for (int j = 1;j < numRx; ++j)
        {
            //dSum += (4*PI * )
        }
    }
    */
    return (((this->x-x) * (this->x-x) + (this->y-y) * (this->y-y)));
}

}; //namespace
