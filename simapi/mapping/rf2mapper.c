#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "../simmap.h"
#include "../simdata.h"
#include "../simapi.h"
#include "../simmapper.h"
#include "../rf2.h"

#include "../../include/rf2data.h"

static int droundint(double d)
{
    return trunc(nearbyint(d));
}

static LapTime rf2_convert_to_simdata_laptime(double rf2_laptime)
{
    if(rf2_laptime <= 0)
    {
        return (LapTime){0, 0, 0};
    }
    LapTime l;
    l.hours = rf2_laptime/60/60;
    l.minutes = rf2_laptime/60-(l.hours*60);
    l.seconds = rf2_laptime-(l.minutes*60);
    l.fraction = (rf2_laptime*1000)-(l.minutes*60000)-(l.seconds*1000);
    return l;
}

static int rf2_phase_to_simdata_flag(int rf2_flag)
{

    int courseflag = 0;
    if(rf2_flag < 5 || rf2_flag > 8)
    {
        courseflag = 0;
    }
    else
    {
        courseflag = rf2_flag - 5;
    }

    return courseflag;
}

static int rf2_flag_to_simdata_flag(int rf2_flag)
{

    int playerflag = 0;
    if(rf2_flag == 6)
    {
        playerflag = 4;
    }

    return playerflag;
}


void map_rfactor2_data(SimData* simdata, SimMap* simmap)
{
    struct rF2Telemetry* telemetry = (struct rF2Telemetry*) simmap->d.rf2.telemetry_map_addr;
    struct rF2Scoring * scoring = (struct rF2Scoring*) simmap->d.rf2.scoring_map_addr;

    // Find the player vehicle.

    int sco = 0; // Index of player vehicle in scoring.
    int veh = 0; // Index of player vehicle in telemetry.

    if (simmap->d.rf2.has_scoring == true) {
        int id = -1;
        for (int i = 0; i < scoring->mScoringInfo.mNumVehicles; i++) {
            if (scoring->mVehicles[i].mControl == 0) {
                sco = i;
                id = scoring->mVehicles[i].mID;
                break;
            }
        }

        if (id != -1) {
            for (int i = 0; i < telemetry->mNumVehicles; i++) {
                if (id == telemetry->mVehicles[i].mID) {
                    veh = i;
                    break;
                }
            }
        }
    }

    // basic telemetry

    simdata->velocity = abs(droundint(3.6 * telemetry->mVehicles[veh].mLocalVel.z));
    simdata->rpms = telemetry->mVehicles[veh].mEngineRPM;
    simdata->gear = telemetry->mVehicles[veh].mGear;
    simdata->maxrpm = droundint(telemetry->mVehicles[veh].mEngineMaxRPM);
    simdata->gas = telemetry->mVehicles[veh].mUnfilteredThrottle;
    simdata->brake = telemetry->mVehicles[veh].mUnfilteredBrake;
    simdata->clutch = telemetry->mVehicles[veh].mUnfilteredClutch;
    simdata->steer = telemetry->mVehicles[veh].mUnfilteredSteering;
    simdata->fuel = telemetry->mVehicles[veh].mFuel;
    simdata->brakebias = telemetry->mVehicles[veh].mRearBrakeBias;
    simdata->handbrake = 0;
    simdata->altitude = 1;

    simdata->gearc[0] = simdata->gear + '0';
    if (simdata->gear < 0)
    {
        simdata->gearc[0] = 'R';
    }
    if (simdata->gear == 0)
    {
        simdata->gearc[0] = 'N';
    }
    simdata->gearc[1] = 0;
    simdata->gear += 1;

    // tyre effects
    //simdata->abs = *(float*) (char*) (a + offsetof(struct rF2Telemetry, mVehicles));
    simdata->tyreRPS[0] = -1 * telemetry->mVehicles[veh].mWheel[0].mRotation;
    simdata->tyreRPS[1] = -1 * telemetry->mVehicles[veh].mWheel[1].mRotation;
    simdata->tyreRPS[2] = -1 * telemetry->mVehicles[veh].mWheel[2].mRotation;
    simdata->tyreRPS[3] = -1 * telemetry->mVehicles[veh].mWheel[3].mRotation;

    simdata->Xvelocity = telemetry->mVehicles[veh].mLocalVel.x;
    simdata->Zvelocity = telemetry->mVehicles[veh].mLocalVel.y;
    simdata->Yvelocity = telemetry->mVehicles[veh].mLocalVel.z;

    rF2Vec3* orix = &telemetry->mVehicles[veh].mOri[0];
    rF2Vec3* oriy = &telemetry->mVehicles[veh].mOri[1];
    rF2Vec3* oriz = &telemetry->mVehicles[veh].mOri[2];

    simdata->worldXvelocity = (orix->x * simdata->Xvelocity) + (orix->z * simdata->Yvelocity) + (orix->y * simdata->Zvelocity);
    simdata->worldYvelocity = (oriz->x * simdata->Xvelocity) + (oriz->z * simdata->Yvelocity) + (oriz->y * simdata->Zvelocity);
    simdata->worldZvelocity = (oriy->x * simdata->Xvelocity) + (oriy->z * simdata->Yvelocity) + (oriy->y * simdata->Zvelocity);

    simdata->Xvelocity = -1 * simdata->Xvelocity;
    simdata->Yvelocity = -1 * simdata->Yvelocity;
    simdata->Zvelocity = -1 * simdata->Zvelocity;

    //advanced ui
    if (simmap->d.rf2.has_scoring == true )
    {
        uint8_t phase = scoring->mScoringInfo.mGamePhase;
        // TODO: will need to track something additional since on session over a value of 8 will still be present when
        // the user has returned to the menu
        if (phase > 2)
        {
            simdata->simstatus = 2;
        }
        else
        {
            simdata->simstatus = 0;
        }
        switch (phase)
        {
            case 0:
            case 1:
            case 2:
            case 3:
            case 4:
            case 9:
                simdata->session = 0;
                break;
            case 5:
            case 6:
            case 7:
            case 8:
                simdata->session = 1;
                break;
            case 10:
            case 11:
            case 12:
            case 13:
                simdata->session = 2;
                break;
            default:
                simdata->session = 0;
                break;
        }


        simdata->tyrewear[0] = telemetry->mVehicles[veh].mWheel[0].mWear;
        simdata->tyrewear[1] = telemetry->mVehicles[veh].mWheel[1].mWear;
        simdata->tyrewear[2] = telemetry->mVehicles[veh].mWheel[2].mWear;
        simdata->tyrewear[3] = telemetry->mVehicles[veh].mWheel[3].mWear;

        simdata->tyretemp[0] = telemetry->mVehicles[veh].mWheel[0].mTireCarcassTemperature;
        simdata->tyretemp[1] = telemetry->mVehicles[veh].mWheel[1].mTireCarcassTemperature;
        simdata->tyretemp[2] = telemetry->mVehicles[veh].mWheel[2].mTireCarcassTemperature;
        simdata->tyretemp[3] = telemetry->mVehicles[veh].mWheel[3].mTireCarcassTemperature;

        for(int k = 0; k<4; k++)
        {
            simdata->tyretemp[k] = simdata->tyretemp[k] - 273.15;
        }

        simdata->braketemp[0] = telemetry->mVehicles[veh].mWheel[0].mBrakeTemp;
        simdata->braketemp[1] = telemetry->mVehicles[veh].mWheel[1].mBrakeTemp;
        simdata->braketemp[2] = telemetry->mVehicles[veh].mWheel[2].mBrakeTemp;
        simdata->braketemp[3] = telemetry->mVehicles[veh].mWheel[3].mBrakeTemp;

        simdata->tyrepressure[0] = telemetry->mVehicles[veh].mWheel[0].mPressure;
        simdata->tyrepressure[1] = telemetry->mVehicles[veh].mWheel[1].mPressure;
        simdata->tyrepressure[2] = telemetry->mVehicles[veh].mWheel[2].mPressure;
        simdata->tyrepressure[3] = telemetry->mVehicles[veh].mWheel[3].mPressure;

        simdata->airtemp = scoring->mScoringInfo.mAmbientTemp;
        simdata->tracktemp = scoring->mScoringInfo.mTrackTemp;

        double trackdist = scoring->mScoringInfo.mLapDist;
        double pos = scoring->mVehicles[sco].mLapDist;
        if(pos < 0)
        {
            pos = (-1 * pos) + .5;
        }
        simdata->tracksamples = ceil(trackdist * 4);
        simdata->playerspline = (pos/trackdist);

        simdata->lap = 1 + telemetry->mVehicles[veh].mLapNumber;
        simdata->position = scoring->mVehicles[sco].mPlace;

        simdata->lastlap = rf2_convert_to_simdata_laptime(scoring->mVehicles[sco].mLastLapTime);
        simdata->bestlap = rf2_convert_to_simdata_laptime(scoring->mVehicles[sco].mBestLapTime);
        simdata->currentlap = rf2_convert_to_simdata_laptime(scoring->mVehicles[sco].mTimeIntoLap);

        simdata->numlaps = scoring->mScoringInfo.mMaxLaps;
        if(simdata->numlaps == INT_MAX)
        {
            simdata->numlaps = 0;
        }
        //simdata->session
        simdata->sectorindex = scoring->mVehicles[sco].mSector;
        //simdata->lastsectorinms
        simdata->playerflag = rf2_flag_to_simdata_flag(scoring->mVehicles[sco].mFlag);
        simdata->courseflag = rf2_phase_to_simdata_flag(scoring->mScoringInfo.mGamePhase);
        simdata->sessiontime = rf2_convert_to_simdata_laptime(telemetry->mVehicles[veh].mElapsedTime);

        // Car and Track
        strncpy(simdata->car, scoring->mVehicles[sco].mVehicleName, 64);
        strncpy(simdata->track, telemetry->mVehicles[veh].mTrackName, 64);

        // Driver
        strncpy(simdata->driver, scoring->mVehicles[sco].mDriverName, 32);

        //Tyre Compound
        strncpy(simdata->tyrecompound, telemetry->mVehicles[veh].mRearTireCompoundName, 18);

        simdata->numcars = telemetry->mNumVehicles;
        int numcars = simdata->numcars;
        if (numcars > MAXCARS)
        {
            numcars = MAXCARS;
        }
        for(int i=0; i<numcars; i++)
        {
            simdata->cars[i].lap = telemetry->mVehicles[i].mLapNumber;
            simdata->cars[i].pos = scoring->mVehicles[i].mPlace;
            uint8_t pitstate = scoring->mVehicles[i].mPitState;
            uint8_t garagestall = scoring->mVehicles[i].mInGarageStall;
            simdata->cars[i].ingarage = false;
            simdata->cars[i].inpitstopped = false;
            simdata->cars[i].inpitentrance = false;
            simdata->cars[i].inpitexit = false;
            if(pitstate == 2)
            {
                simdata->cars[i].inpitentrance = true;
            }
            if(pitstate == 3)
            {
                simdata->cars[i].inpitstopped = true;
            }
            if(pitstate == 4)
            {
                simdata->cars[i].inpitexit = true;
            }
            if(pitstate >= 2)
            {
                simdata->cars[i].inpit = true;
            }
            if(pitstate == 2 || pitstate == 4)
            {
                simdata->cars[i].inpitlane = true;
            }
            if(garagestall > 0)
            {
                simdata->cars[i].ingarage = true;
            }
            if(simdata->cars[i].ingarage == true)
            {
                simdata->cars[i].inpit = true;
            }

            simdata->cars[i].lastlap = rf2_convert_to_simdata_laptime(scoring->mVehicles[i].mLastLapTime);
            simdata->cars[i].bestlap = rf2_convert_to_simdata_laptime(scoring->mVehicles[i].mBestLapTime);

            strncpy(simdata->cars[i].driver, scoring->mVehicles[i].mDriverName, 32);

            strncpy(simdata->cars[i].car, scoring->mVehicles[i].mVehicleName, 64);

            simdata->cars[i].xpos = telemetry->mVehicles[i].mPos.x;
            simdata->cars[i].zpos = telemetry->mVehicles[i].mPos.y;
            simdata->cars[i].ypos = telemetry->mVehicles[i].mPos.z;
        }

        simdata->worldposx = telemetry->mVehicles[veh].mPos.x;
        simdata->worldposz = telemetry->mVehicles[veh].mPos.y;
        simdata->worldposy = telemetry->mVehicles[veh].mPos.z;

        SetProximityData(simdata, numcars, 1);
    }

}
