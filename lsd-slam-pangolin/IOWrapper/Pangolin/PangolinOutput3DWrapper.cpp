/*
 * PangolinOutput3DWrapper.cpp
 *
 *  Created on: 17 Oct 2014
 *      Author: thomas
 */

#include "PangolinOutput3DWrapper.h"

#include "util/SophusUtil.h"
#include "util/settings.h"
#include "DataStructures/Frame.h"
#include "GlobalMapping/KeyFrameGraph.h"
#include "sophus/sim3.hpp"
#include "GlobalMapping/g2oTypeSim3Sophus.h"

namespace lsd_slam
{
    
    PangolinOutput3DWrapper::PangolinOutput3DWrapper(int width, int height, GUI & gui)
    : width(width),
    height(height),
    gui(gui),
    publishLvl(0)
    {
        
    }
    
    PangolinOutput3DWrapper::~PangolinOutput3DWrapper()
    {
        
    }
    
    void PangolinOutput3DWrapper::updateImage(unsigned char * data, unsigned char * data2)
    {
        gui.updateImage(data, data2);
    }
    
    void PangolinOutput3DWrapper::publishKeyframe(Frame* f)
    {
        Keyframe * fMsg = new Keyframe;
        
        boost::shared_lock<boost::shared_mutex> lock = f->getActiveLock();
        
        fMsg->id = f->id();
        fMsg->time = f->timestamp();
        fMsg->isKeyframe = true;
        
        int w = f->width(publishLvl);
        int h = f->height(publishLvl);
        
        fMsg->camToWorld = f->getScaledCamToWorld().cast<float>();
        
        fMsg->fx = f->fx(publishLvl);
        fMsg->fy = f->fy(publishLvl);
        fMsg->cx = f->cx(publishLvl);
        fMsg->cy = f->cy(publishLvl);
        
        fMsg->width = w;
        fMsg->height = h;
        
        fMsg->pointData = new unsigned char[w * h * sizeof(InputPointDense)];
        
        InputPointDense * pc = (InputPointDense*)fMsg->pointData;
        
        const float* idepth = f->idepth(publishLvl);
        const float* idepthVar = f->idepthVar(publishLvl);
        const float* color = f->image(publishLvl);
        
        for(int idx = 0;idx < w * h; idx++)
        {
            pc[idx].idepth = idepth[idx];
            pc[idx].idepth_var = idepthVar[idx];
            pc[idx].color[0] = color[idx];
            pc[idx].color[1] = color[idx];
            pc[idx].color[2] = color[idx];
            pc[idx].color[3] = color[idx];
        }
        
        lock.unlock();
        
        gui.addKeyframe(fMsg);
        
        writePointCloud(f);
    }
    
    void PangolinOutput3DWrapper::writePointCloud(Frame* f) {
        printf("Finished processing key frame %d\n", f->id());
        
        fx = f->fx(publishLvl);
        fy = f->fy(publishLvl);
        cx = f->cx(publishLvl);
        cy = f->cy(publishLvl);
        
        fxi = 1/fx;
        fyi = 1/fy;
        cxi = -cx / fx;
        cyi = -cy / fy;
        
        int width= f->width(publishLvl);
        int height = f->height(publishLvl);
        
        const float* idepth = f->idepth(publishLvl);
        const float* idepthVar = f->idepthVar(publishLvl);
        
        Sophus::Sim3f camToWorld = f->getScaledCamToWorld().cast<float>();
        Keyframe::MyVertex* tmpBuffer = new Keyframe::MyVertex[width*height];
        
        int num = 0;
        int total = 0, displayed = 0;
        
        my_scaledTH = scaledDepthVarTH;
        my_absTH = absDepthVarTH;
        my_scale = camToWorld.scale();
        my_minNearSupport = minNearSupport;
        my_sparsifyFactor = sparsifyFactor;
        
        for(int y=1; y<height-1; y++)
            for(int x=1; x<width-1; x++) {
                if(idepth[x+y*width] <= 0) continue;
                total++;
                
                if(my_sparsifyFactor > 1 && rand() % my_sparsifyFactor != 0) continue;
                
                float depth = 1 / idepth[x+y*width];
                float depth4 = depth*depth; depth4*= depth4;
                
                
                if(idepthVar[x+y*width] * depth4 > my_scaledTH)
                    continue;
                
                if(idepthVar[x+y*width] * depth4 * my_scale*my_scale > my_absTH)
                    continue;
                
                Sophus::Vector3f pt = camToWorld * (Sophus::Vector3f((x*fxi + cxi), (y*fyi + cyi), 1) * depth);
                tmpBuffer[num].point[0] = pt[0];
                tmpBuffer[num].point[1] = pt[1];
                tmpBuffer[num].point[2] = pt[2];
                
                num++;
            }
        
        std::ofstream output;
        
        if (firstKeyframe) {
            output.open("pointCloud.xyz");
            output.precision(6);
            output << "3" << std::endl;
            
            firstKeyframe = false;
        }
        else {
            output.open("pointCloud.xyz", std::fstream::app);
            output.precision(6);
        }
        
        for(int i=0; i<num; i++) {
            output << tmpBuffer[i].point[0] << " " << tmpBuffer[i].point[1] << " "  << tmpBuffer[i].point[2] << "\n";
        }
        
        output.close();
        
    }
    
    void PangolinOutput3DWrapper::publishTrackedFrame(Frame* kf)
    {
        //    lsd_slam_viewer::keyframeMsg fMsg;
        //
        //
        //    fMsg.id = kf->id();
        //    fMsg.time = kf->timestamp();
        //    fMsg.isKeyframe = false;
        //
        //
        //    memcpy(fMsg.camToWorld.data(),kf->getScaledCamToWorld().cast<float>().data(),sizeof(float)*7);
        //    fMsg.fx = kf->fx(publishLvl);
        //    fMsg.fy = kf->fy(publishLvl);
        //    fMsg.cx = kf->cx(publishLvl);
        //    fMsg.cy = kf->cy(publishLvl);
        //    fMsg.width = kf->width(publishLvl);
        //    fMsg.height = kf->height(publishLvl);
        //
        //    fMsg.pointcloud.clear();
        //
        //    liveframe_publisher.publish(fMsg);
        //
        //
        //    SE3 camToWorld = se3FromSim3(kf->getScaledCamToWorld());
        //
        //    geometry_msgs::PoseStamped pMsg;
        //
        //    pMsg.pose.position.x = camToWorld.translation()[0];
        //    pMsg.pose.position.y = camToWorld.translation()[1];
        //    pMsg.pose.position.z = camToWorld.translation()[2];
        //    pMsg.pose.orientation.x = camToWorld.so3().unit_quaternion().x();
        //    pMsg.pose.orientation.y = camToWorld.so3().unit_quaternion().y();
        //    pMsg.pose.orientation.z = camToWorld.so3().unit_quaternion().z();
        //    pMsg.pose.orientation.w = camToWorld.so3().unit_quaternion().w();
        //
        //    if (pMsg.pose.orientation.w < 0)
        //    {
        //        pMsg.pose.orientation.x *= -1;
        //        pMsg.pose.orientation.y *= -1;
        //        pMsg.pose.orientation.z *= -1;
        //        pMsg.pose.orientation.w *= -1;
        //    }
        //
        //    pMsg.header.stamp = ros::Time(kf->timestamp());
        //    pMsg.header.frame_id = "world";
        //    pose_publisher.publish(pMsg);
    }
    
    void PangolinOutput3DWrapper::publishKeyframeGraph(KeyFrameGraph* graph)
    {
        graph->keyframesAllMutex.lock_shared();
        
        int num = (int) graph->keyframesAll.size();
        
        unsigned char * buffer = new unsigned char[num * sizeof(GraphFramePose)];
        
        GraphFramePose* framePoseData = (GraphFramePose*)buffer;
        
        for(unsigned int i = 0; i < graph->keyframesAll.size(); i++)
        {
            framePoseData[i].id = graph->keyframesAll[i]->id();
            memcpy(framePoseData[i].camToWorld, graph->keyframesAll[i]->getScaledCamToWorld().cast<float>().data(), sizeof(float) * 7);
        }
        
        graph->keyframesAllMutex.unlock_shared();
        
        gui.updateKeyframePoses(framePoseData, num);
        
        delete [] buffer;
    }
    
    void PangolinOutput3DWrapper::publishTrajectory(std::vector<Eigen::Matrix<float, 3, 1>> trajectory, std::string identifier)
    {
        //TODO
    }
    
    void PangolinOutput3DWrapper::publishTrajectoryIncrement(Eigen::Matrix<float, 3, 1> pt, std::string identifier)
    {
        //TODO
    }
    
    void PangolinOutput3DWrapper::publishDebugInfo(Eigen::Matrix<float, 20, 1> data)
    {
        //TODO
    }
    
}
