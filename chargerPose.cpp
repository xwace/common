#include "storeChargerPoses.h"
#include "handle_line_laser.h"

namespace CHARGERPOSES
{  
	class CThread {
	public:
	    CThread() {};
	
	    virtual ~CThread() {};
	
	//		int thread_wait(void);
	//		int thread_wait(int ms);
	//		bool thread_signal(void *data);
	//		bool thread_del();
	//		bool start();
	    bool start(const char *name) {
		printf("\t\tthread: %s \t~ \t%u\n", name, std::this_thread::get_id());
		int ret;
		pthread_attr_t thread_attr;
	
		if (m_bCreate) {
		    return true;
		}
	
		ret = pthread_create(&handle, NULL, StaticThreadFunc, this);
	
		if (ret != 0) {
		    return false;
		}
	
		m_bCreate = true;
	
	    };
	
	    virtual void run() {};
	
	    void join() {
		int *thread_ret = NULL;
		pthread_join(handle, (void **) &thread_ret);
	    };
	    void cancel();
	    void pexit();
	
	private:
	    static void *StaticThreadFunc(void *arg) {
		CThread *pthis = static_cast<CThread *>(arg);
		pthis->run();
	    };
	    pthread_t handle;
	    bool m_bCreate;
	    int thread_end = 0;
	
	protected:
	    std::condition_variable thread_cv;
	    std::mutex thread_mtx;
			virtual bool set_signal(void * data){};
			virtual bool check_signal(){};
	};
	
	bool flag{false};
	
　　　class chargerRadPoses : public CThread {
	    static chargerRadPoses m_Instance;
	
	    void run() override;
	
	public:
	    chargerRadPoses() {};
	
	    ~chargerRadPoses() {};
	
	    static chargerRadPoses &GetInstance() { return m_Instance; }//返回类本身
	    void startChargerDetect();
	
	    void stopChargerDetect();
	};

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

        auto poseToCoordinate=[](const SLAM::RealtimePosition &Pose, NodeInfo &Node, const MapInfo &mMapInfo){
            Node.mRow = mMapInfo.OriginNode.mRow - (100 * Pose.m_PoseX + (float)5 / 2) / 5.0;
		    Node.mCol = mMapInfo.OriginNode.mCol - (100 * Pose.m_PoseY + (float)5 / 2) / 5.0;
        };

        ############################################################################################################    
        std::vector<cv::Point>pts;
        for(auto pt:lidarInfo.mLaserData){
            NodeInfo node;
            SLAM::RealtimePosition pose;
            pose.m_PoseX=pt.nDist_MM*(1e-3)*std::cos(pt.nAngle/180*CV_PI);
            pose.m_PoseY=pt.nDist_MM*(1e-3)*std::sin(pt.nAngle/180*CV_PI);
            if (pose.m_PoseX != 0 and pose.m_PoseY != 0)
            {
                poseToCoordinate(pose, node, map);
                pts.emplace_back(node.mCol,node.mRow);
            }
        }

        cv::Mat pts_img=cv::Mat(pts).reshape(1,pts.size());
        double h,w,minus_h,minus_w;
        minMaxIdx(pts_img.col(0),&minus_w,&w);
        minMaxIdx(pts_img.col(1),&minus_h,&h);
        cv::Mat output(h+std::abs(minus_w)+1,w+std::abs(minus_h)+2,0,Scalar(0));
        for(auto pt:pts){
            if(cv::Rect(0,0,w,h).contains(pt)){
                output.at<uchar>(pt.y+minus_h,pt.x+minus_w)=255;
            }
        }

        static int cnt=1;
        cnt++;
        string name=std::to_string(cnt);
        cv::imwrite("/home/robot/business/log/lidar/"+name+".png",output);
        ############################################################################################################  
    }

}
