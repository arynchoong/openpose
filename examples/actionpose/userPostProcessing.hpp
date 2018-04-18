#ifndef OPENPOSE_EXAMPLES_TUTORIAL_USER_POST_PROCESSING_HPP
#define OPENPOSE_EXAMPLES_TUTORIAL_USER_POST_PROCESSING_HPP

#include <openpose/core/common.hpp>
#include <openpose/core/cvMatToOpInput.hpp>
#include <openpose/pose/enumClasses.hpp>
#include <openpose/utilities/keypoint.hpp>
#include <math.h>

#define PI 3.14159265

namespace op
{
    /**
     * Add your class description here.
     */
    class UserPostProcessing
    {
    public:
        /**
         * Add your constructor description here.
         */
        UserPostProcessing();

        /**
         * Add your initializationOnThread description here.
         * This is equivalent to the constructor, but it is run in the thread where this function will operate
         */
        void initializationOnThread();

        /**
         * Add your information about what your class does and their inputs/outputs.
         * @param output is the modified cv::Mat.
         * @param input is the input cv::Mat.
         * @return If it is not void, add returning information here.
         */
        std::string getBodyPoseAction(Array<float>& poseKeypoints);
        std::string getPoseAction(Array<float>& poseKeypoints);
        void setPoseActionToOut(cv::Mat& output, Array<float>& poseKeypoints);

    private:
        /**
         * Description of your variable here.
         */
        PoseModel mPoseModel;
    };
}





// Implementation
namespace op
{
    UserPostProcessing::UserPostProcessing()
    {
        try
        {
            // TODO: CHANGE THIS HARDCODING !!
            mPoseModel = PoseModel::COCO_18;
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }

    void UserPostProcessing::initializationOnThread()
    {
        try
        {
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }

    unsigned int verticalAngle(int x1, int y1, int x2, int y2)
    {
        unsigned int mAngle;
        mAngle = abs(atan2((x2-x1), (y2-y1)) * 180 / PI);
        return mAngle;
    }

    std::string UserPostProcessing::getBodyPoseAction(Array<float>& poseKeypoints)
    {
        try
        {
            const auto* posePtr = &poseKeypoints.at(0);
            enum eParts{NECK=0, RHIP, LHIP, RKNEE, LKNEE, EPARTS_END};
            std::string mParts[5] = {"Neck", "RHip", "LHip", "RKnee", "LKnee"};
            unsigned int mIdx[5];
            double mXParts[5], mYParts[5], mHeight, mWidth;
            int mSmallestX=0, mLargestX=0, mSmallestY=0, mLargestY=0;
            int mRatio = 30;
            int mTorsoAngle, mLLegAngle, mRLegAngle;
            float mDistNeckHip=0, mDistLHipKnee=0, mDistRHipKnee=0;

            for (int i = 0; i<EPARTS_END; i++)
            {
                mIdx[i] = poseBodyPartMapStringToKey(mPoseModel, mParts[i]);
                mXParts[i] = posePtr[mIdx[i]*3];
                mYParts[i] = posePtr[mIdx[i]*3+1];
                
                // for calculating height and length
                if(mXParts[i]!=0 || mYParts[i]!=0)
                {
                    if(mSmallestX==0 && mLargestX==0 && mSmallestY==0 && mLargestY==0)
                        {
                            mSmallestX=mXParts[i]; mLargestX=mXParts[i]; 
                            mSmallestY=mYParts[i]; mLargestY=mYParts[i];
                        }
                    else
                    {
                        if(mSmallestX > mXParts[i]) mSmallestX = mXParts[i];
                        if(mLargestX < mXParts[i]) mLargestX = mXParts[i];
                        if(mSmallestY > mYParts[i]) mSmallestY = mYParts[i];
                        if(mLargestY < mYParts[i]) mLargestY = mYParts[i];
                    }
                }
                else if(i == NECK) // cannot continue without neck keypoint
                    return "";
            }
            // set height and width
            mHeight = mLargestY - mSmallestY;
            mWidth = mLargestX - mSmallestX;
            // cannot continue without key body parts keypoints
            if(mXParts[RHIP]==0 && mYParts[RHIP]==0 && mXParts[LHIP]==0 && mYParts[LHIP]==0)
                return "";
            if(mXParts[RKNEE]==0 && mYParts[RKNEE]==0 && mXParts[LKNEE]==0 && mYParts[LKNEE]==0)
                return "";
            
            // Calculate key distances
            if(mXParts[RHIP]==0 && mYParts[RHIP]==0)
                mDistNeckHip = getDistance(poseKeypoints, 0, mIdx[NECK], mIdx[LHIP]);
            else if(mXParts[LHIP]==0 && mYParts[LHIP]==0)
                mDistNeckHip = getDistance(poseKeypoints, 0, mIdx[NECK], mIdx[RHIP]);
            else // average
                mDistNeckHip = (getDistance(poseKeypoints, 0, mIdx[NECK], mIdx[LHIP]) + getDistance(poseKeypoints, 0, mIdx[NECK], mIdx[RHIP])) / 2;
            if(mXParts[RKNEE]!=0 || mYParts[RKNEE]!=0)
                mDistRHipKnee = getDistance(poseKeypoints, 0, mIdx[RKNEE], mIdx[RHIP]);
            if(mXParts[LKNEE]!=0 && mYParts[LKNEE]!=0)
                mDistLHipKnee = getDistance(poseKeypoints, 0, mIdx[LKNEE], mIdx[LHIP]);
            
            // Calculate pair part angles
            mTorsoAngle = (verticalAngle(mXParts[NECK], mYParts[NECK], mXParts[RHIP], mYParts[RHIP]) + verticalAngle(mXParts[NECK], mYParts[NECK], mXParts[LHIP], mYParts[LHIP])) / 2; 
            mLLegAngle = verticalAngle(mXParts[RHIP], mYParts[RHIP], mXParts[RKNEE], mYParts[RKNEE]);
            mRLegAngle = verticalAngle(mXParts[LHIP], mYParts[LHIP], mXParts[LKNEE], mYParts[LKNEE]);
            
            if(mTorsoAngle < 16) // torso straight
            {
                // legs straight
                if(mLLegAngle < 30 && mRLegAngle < 30)
                {
                    // Check for standing position
                    if (((mWidth*100/mHeight) < mRatio) &&
                        ((mHeight * 0.8) < (mDistNeckHip+mDistRHipKnee) < (mHeight * 1.2)) &&
                        ((mHeight * 0.8) < (mDistNeckHip+mDistLHipKnee) < (mHeight * 1.2)))
                            return "Standing";
                }
            }
            // else lying, sitting, bending
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
            return "";
        }
        return "";
    }

    std::string UserPostProcessing::getPoseAction(Array<float>& poseKeypoints)
    {
        try
        {
            std::string mPoseAction = "";
            if(poseKeypoints.empty())
            {
                return mPoseAction;
            }
            // Get body pose according to parameter
            mPoseAction = getBodyPoseAction(poseKeypoints);
            // Check if Standing
            // Get length of neck-hip + hip-knee = within stddev 10* of neck-knee 
            // check for x or y axis is longer, and within ratio of rectangle, then standing or lying
            
            // if one only one is within angle: if larger than 30%, check for kick or walk/run
            
            // else check for bow / sit 
            // Default pose set
            return mPoseAction;
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
            return "";
        }
    }

    void UserPostProcessing::setPoseActionToOut(cv::Mat& output, Array<float>& poseKeypoints)
    {
        std::string mPoseAction = "";
        try
        {
            if(output.empty())
                error("Wrong imput cv:mat output", __LINE__, __FUNCTION__, __FILE__);
            // border relative to size
            const auto borderMargin = intRound(fastMax(output.cols, output.rows) * 0.025);
            // Used colors
            const cv::Scalar red{10, 10, 255};

            // Get pose action
            mPoseAction = getPoseAction(poseKeypoints);

            // Draw Action on output
            putTextOnCvMat(output, "OpenPose : " + mPoseAction,
                           {borderMargin, borderMargin}, red, false, output.cols);
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }

    // COMPILE_TEMPLATE_DATUM(UserPostProcessing);
}

#endif // OPENPOSE_EXAMPLES_TUTORIAL_USER_POST_PROCESSING_HPP
