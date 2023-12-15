#include "storeChargerPoses.h"
#include "handle_line_laser.h"

namespace CHARGERPOSES
{  
    chargerRadPoses chargerRadPoses::m_Instance;

    void chargerRadPoses::startChargerDetect(){
        GetInstance().start("chargerRadPoses detect");
    }

    void chargerRadPoses::stopChargerDetect(){
        stopDetect = true;
        VECTOR_2D().swap(rads);
        GetInstance().join();
    }

    void chargerRadPoses::run(){
        while(true){
            usleep(1e6);
            if (stopDetect) break;

            auto detect_rad = m_Detect.getRads();
            std::unique_lock<std::mutex> lock(thread_mtx);
            if (!detect_rad.empty()) VECTOR_2D(detect_rad).swap(rads);
        }
    }

    VECTOR_2D chargerRadPoses::getRads(){
        std::unique_lock<std::mutex> lock(thread_mtx);
        if (rads.empty())
        {
            return{};
        }
        return rads;
    }

    VECTOR_2D chargerRadDetect::getRads(){
        MapInfo map;
        MAP_UPDATE::CMapUpdate::GetInstance().GetMap(eGlobalMap, map);
        
        NodeInfo curNode;
        SLAM::RealtimePosition curPose;
        SLAM::CSlam::GetInstance().GetPoseEstimate(curPose);
        MAP_UPDATE::CMapUpdate::GetInstance().PoseToCoordinateBk(curPose, curNode, map);

        LaserInfo lidarInfo;
        HANDLINELASER::CHandleLineLaser::GetInstance().GetLidar(lidarInfo);       
    }

}
