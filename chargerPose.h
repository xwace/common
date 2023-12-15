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
        LaserInfo mLaserInfo;
        mLaserInfo.mLaserData.resize(360);
        HANDLINELASER::CHandleLineLaser::GetInstance().GetLidar(mLaserInfo);

        std::vector<std::vector<GridOccupyType>> mMapData;
        MAP_UPDATE::CMapUpdate::GetInstance().GetMap(mMapData);

        SLAM::RealtimePosition cur_pose;
        NodeInfo cur_node;
        MAP_UPDATE::MapParam mMapParam_RAD;
        auto mapInfo = MAP_UPDATE::CMapUpdate::GetInstance().getJpsBaseMap(mMapParam_RAD);
        SLAM::CSlam::GetInstance().GetPoseEstimate(cur_pose);
        MAP_UPDATE::CMapUpdate::GetInstance().PoseToCoordinateBk(cur_pose, cur_node, mapInfo);


        
    }

}
