/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/
#define FMT_HEADER_ONLY
#include "fmt/format.h"

#ifndef MOCAP_H
#define MOCAP_H

#include <mutex>
#include <map>
#include <sophus/se3.hpp>
#include <NatNet/NatNetCAPI.h>
#include <NatNet/NatNetClient.h>

using namespace std;

void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData);    // receives data from the server
void NATNET_CALLCONV ServerDiscoveredCallback( const sNatNetDiscoveredServer* pDiscoveredServer, void* pUserContext );

class Mocap
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Mocap(std::vector<int> &id);

    int ConnectClient();
    NatNetClient* GetNatNetClient() {return g_pClient;}
    std::vector< sNatNetDiscoveredServer >& GetDiscoveredServers() {return g_discoveredServers;}
    sServerDescription& GetServerDescription() {return g_serverDescription;}

    void SetRigidBodyPose(Eigen::Quaternionf q, Eigen::Vector3f t, int32_t id, double time, int idFrame);
    void GetRigidBodyPose(Sophus::SE3d& pF_, double& timeF_, int& idF_, Sophus::SE3d& pL_, double& timeL_, int& idL_);
    std::map<int, bool>& GetVaild() {return valid;}
    std::map<int, Sophus::SE3f>& GetPose() {return pose;}

    double& GetFrameTime() {return time;}
    void SetFrameTime(double t) {time = t;}
    int& GetFrameId() {return idFrame;}
    void SetFrameId(int id) {idFrame = id;}
    std::mutex mMutexPose;

private:
    NatNetClient* g_pClient; 
    int g_analogSamplesPerMocapFrame;  
    char g_discoveredMulticastGroupAddr[kNatNetIpv4AddrStrLenMax];
    std::vector< sNatNetDiscoveredServer > g_discoveredServers;
    sNatNetClientConnectParams g_connectParams;
    sServerDescription g_serverDescription;

    int numBody;
    std::map<int, Sophus::SE3f> pose;
    std::map<int, bool> valid;
    double time;
    int idFrame;

};




#endif // VIEWER_H
	

