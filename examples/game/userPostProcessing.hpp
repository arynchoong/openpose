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
        std::string processState(std::string mPoseAction);
        void setPoseActionToOut(cv::Mat& output, Array<float>& poseKeypoints);
        int getUpperBodyPose(Array<float>& poseKeypoints, int idxPerson);
        int getUpperPose(Array<float>& poseKeypoints);
        std::string getMessage(int gameState);
        std::string getMessageLine2(int gameState);
        std::string processGameState(int upperPose);
	enum GAME_STATES
	{
	    G_IDLE,
	    G_INIT,
	    G_START,
	    G_DRINK,
	    G_MILK,
	    G_SUGAR,
	    G_SERVE,
	    G_END
	};
	enum DRINK
	{
	    D_KOPI_KOSONG,
	    D_KOPI_O,
	    D_KOPI,
	    D_TEH_KOSONG,
	    D_TEH_O,
	    D_TEH_TARIK
	};
	enum GAME_POSE
	{
	    P_IDLE,
	    P_FRONT,
	    P_RIGHT,
	    P_LEFT,
	    P_BOTH
	};
    private:
        /**
         * To keep track of the last pose action.
         * And how many frames with no pose.
         */
        std::string mLastPoseAction;
        int mNumNoPoseFrames;
        PoseModel mPoseModel;
        int mGameState;
        int mDrink;
	int mNumGameFrames;
	int mLastGamePose;
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
            mLastPoseAction = "";
            mNumNoPoseFrames = 0;
            mGameState = G_IDLE;
            mDrink = D_KOPI;
            mLastGamePose = P_IDLE;
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
    
    int getAngleABC(int x1, int y1, int x2, int y2, int x3, int y3)
    {
        int x12 = x2 - x1;
        int y12 = y2 - y1;
        int x32 = x2 - x3;
        int y32 = y2 - y3;

        float dot = (x12 * x32 + y12 * y32); // dot product
        float cross = (x12 * y32 - y12 * x32); // cross product

        float alpha = atan2(cross, dot);

        return (int) floor(alpha * 180. / PI + 0.5);
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
                    if(((mWidth*100/mHeight) < mRatio) &&
                        ((mHeight * 0.8)<(mDistNeckHip+mDistRHipKnee)) && 
			((mDistNeckHip+mDistRHipKnee)<(mHeight * 1.2)) &&
                        ((mHeight * 0.8) < (mDistNeckHip+mDistLHipKnee)) &&
			((mDistNeckHip+mDistLHipKnee) < (mHeight * 1.2)))
                            return "Standing";
                }
                else if(mLLegAngle>45 && mRLegAngle>45)
                {
                    return "Sitting";
                }
            }
            else if(mTorsoAngle > 60)
                return "Bending";
            else
            {
                if(mLLegAngle < 30 && mRLegAngle < 30)
                    return "Bending";
            }
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

    float getRatio(float dist1, float dist2)
    {
        float larger, smaller;
        if (dist1> dist2)
        {
	    larger = dist1;
            smaller = dist2;
        }
        else
        {
	    larger = dist2;
            smaller = dist1;
        }
	return (1.0-(smaller/larger));
    }

    int UserPostProcessing::getUpperBodyPose(Array<float>& poseKeypoints, int idxPerson)
    {
        try
        {
            const auto* posePtr = &poseKeypoints.at(idxPerson*poseKeypoints.getSize(1)*poseKeypoints.getSize(2));
            enum eParts{NOSE=0, NECK, RSHOULDER, RELBOW, RWRIST, LSHOULDER, LELBOW, LWRIST, REAR, LEAR, EPARTS_END};
            std::string mParts[10] = {"Nose","Neck","RShoulder","RElbow","RWrist", 
                            "LShoulder","LElbow","LWrist", "REar", "LEar"};
            unsigned int mIdx[10];
            double mXParts[10], mYParts[10];
            int mShouldersAngle=0, mLHandAngle=0, mRHandAngle=0;
            float mDistRShoulder=0, mDistLShoulder=0, mDistREar=0, mDistLEar=0;

            for (int i = 0; i<EPARTS_END; i++)
            {
                mIdx[i] = poseBodyPartMapStringToKey(mPoseModel, mParts[i]);
                mXParts[i] = posePtr[mIdx[i]*3];
                mYParts[i] = posePtr[mIdx[i]*3+1];
            }
            // cannot continue without shoulders keypoint
            if(mXParts[NOSE]==0 || mXParts[NECK]==0 || mXParts[RSHOULDER]==0 || 
               mXParts[LSHOULDER]==0 || mXParts[LELBOW]==0 || mXParts[RELBOW]==0 || 
               mXParts[LEAR]==0 || mXParts[REAR]==0) 
                return P_IDLE;
            // Calculate key distances
            if(mXParts[REAR]!=0 && mYParts[REAR]!=0)
                mDistREar = getDistance(poseKeypoints, idxPerson, mIdx[NOSE], mIdx[REAR]);
            if(mXParts[LEAR]!=0 && mYParts[LEAR]!=0)
                mDistLEar   = getDistance(poseKeypoints, idxPerson, mIdx[NOSE], mIdx[LEAR]);
            if(mXParts[RSHOULDER]!=0 && mYParts[RSHOULDER]!=0)
                mDistRShoulder = getDistance(poseKeypoints, idxPerson, mIdx[NECK], mIdx[RSHOULDER]);
            if(mXParts[LSHOULDER]!=0 && mYParts[LSHOULDER]!=0)
                mDistLShoulder = getDistance(poseKeypoints, idxPerson, mIdx[NECK], mIdx[LSHOULDER]);
            
            // Calculate pair part angles
            mShouldersAngle = verticalAngle(mXParts[RSHOULDER], mYParts[RSHOULDER], mXParts[LSHOULDER], mYParts[LSHOULDER]); 
            if (mXParts[LWRIST]!=0)
                mLHandAngle = verticalAngle(mXParts[LELBOW], mYParts[LELBOW], mXParts[LWRIST], mYParts[LWRIST]);
            if (mXParts[RWRIST]!=0)
                mRHandAngle = verticalAngle(mXParts[RELBOW], mYParts[RELBOW], mXParts[RWRIST], mYParts[RWRIST]);
            
            // Face front
            if(((75 < mShouldersAngle) && (mShouldersAngle < 105)) && 
               ((getRatio(mDistRShoulder,mDistLShoulder))<0.3) && 
               ((getRatio(mDistREar,mDistLEar))<0.5)) 
            {
                
                if((mLHandAngle!=0) && ((135 < mLHandAngle) && (mLHandAngle < 225)))
                {// Left Hand raised
                    if((mRHandAngle!=0) && ((135 < mRHandAngle) && (mRHandAngle < 225)))
                    {// Right Hand raised
                        return P_BOTH;
                    }
                    return P_LEFT;
                }
                else if((mRHandAngle!=0) && ((135 < mRHandAngle) && (mRHandAngle < 225)))
                {// Right Hand raised
                    return P_RIGHT;
                }
                return P_FRONT;
            }
            else
            {
                return P_IDLE;
            }
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
            return P_IDLE;
        }
    }

    int UserPostProcessing::getUpperPose(Array<float>& poseKeypoints)
    {
        try
        {
            int mIdxBiggest;
            if(poseKeypoints.empty())
            {
                return P_IDLE;
            }
            mIdxBiggest = getBiggestPerson(poseKeypoints,0);
            // Get Upper body pose according to parameter
            return getUpperBodyPose(poseKeypoints, mIdxBiggest);
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
            return P_IDLE;
        }
    }

    std::string UserPostProcessing::processState(std::string mPoseAction)
    {
        if (mPoseAction.length()>0)
        {
            mLastPoseAction = mPoseAction;
            mNumNoPoseFrames=0;
        }
        else if(mNumNoPoseFrames>8)
        {
            mLastPoseAction = "";
        }
        else 
        {
            mNumNoPoseFrames++;
        }
        
        return mLastPoseAction;
    }

    std::string UserPostProcessing::getMessage(int gameState)
    {
	switch(gameState)
	{
	    case G_IDLE:
		return "";
	    case G_INIT:
		return "Hello!";
	    case G_START:
		return "Would you like a drink?";
	    case G_DRINK:
		return "Would you like coffee or tea?";
	    case G_MILK:
		if (mDrink == D_KOPI_KOSONG)
                    return "Would you like milk?";
		else 
                    return "Would you like milk?";
	    case G_SUGAR:
		if ((mDrink == D_KOPI_KOSONG) || (mDrink == D_KOPI_O))
                    return "Would you like sugar?";
		else 
                    return "Would you like sugar?";
	    case G_SERVE:
                switch (mDrink)
                {
                    case D_KOPI_KOSONG:
                        return "Enjoy your KOPI KOSONG !";
                    case D_KOPI_O:
                        return "Enjoy your KOPI O !";
                    case D_KOPI:
                        return "Enjoy your KOPI !";
                    case D_TEH_KOSONG:
                        return "Enjoy your TEH KOSONG !";
                    case D_TEH_O:
                        return "Enjoy your TEH O !";
                    case D_TEH_TARIK:
                        return "Enjoy your TEH TARIK !";
                    default:
                        return "Enjoy your COFFEE !";
                }
	    default:
		mGameState = G_IDLE;
		return "";
	}
	return "";
    }

    std::string UserPostProcessing::getMessageLine2(int gameState)
    {
	switch(gameState)
	{
	    case G_START:
		return "Please raise your hand for YES.";
	    case G_DRINK:
		return "Please raise left hand for COFFEE and right hand for TEA.";
	    case G_MILK:
	    case G_SUGAR:
		return "Please raise left hand for NO and right hand for YES.";
	    case G_IDLE:
	    case G_INIT:
	    case G_SERVE:
	    default:
		return "";
	}
	return "";
    }

    std::string UserPostProcessing::processGameState(int upperPose)
    {
	// Track delay on change of state
	if ((mGameState==G_IDLE) && (upperPose==P_IDLE))
	{
	    mNumGameFrames=0;
	    return getMessage(mGameState);
	}
	else if(mLastGamePose == upperPose) // no change of state
	{
	    mNumGameFrames++;
	    return getMessage(mGameState);
	}
	else if ((mNumGameFrames<5) || ((mGameState == G_SERVE) && (mNumGameFrames<8)))
	{ // changed pose, set delay
	    mLastGamePose = upperPose;
	    mNumGameFrames++;
	    return getMessage(mGameState);
	}
	mLastGamePose = upperPose;
	mNumGameFrames=0;
        switch (upperPose)
	{
	    case P_IDLE:
	        mGameState = G_IDLE;
	        break;
	    case P_FRONT:
	        if(mGameState==G_IDLE)
		    mGameState=G_INIT;
	        break;
	    case P_RIGHT:
		switch (mGameState)
		{
		    case G_INIT:
                        mGameState = G_START;
                        break;
		    case G_START:
                        mDrink = D_KOPI_KOSONG;
                        mGameState = G_DRINK;
                        break;
		    case G_DRINK:
                        mDrink = D_TEH_KOSONG;
                        mGameState = G_MILK;
                        break;
		    case G_MILK:
                        if(mDrink == D_KOPI_KOSONG)
                            mDrink = D_KOPI_O;
                        else
                            mDrink = D_TEH_O;
                        mGameState = G_SUGAR;
                        break;
		    case G_SUGAR:
                        if(mDrink == D_KOPI_KOSONG)
                            mDrink = D_KOPI_O;
                        else if(mDrink == D_KOPI_O)
                            mDrink = D_KOPI;
                        else if(mDrink == D_TEH_KOSONG)
                            mDrink = D_TEH_O;
                        else
                            mDrink = D_TEH_TARIK;
                        mGameState = G_SERVE;
                        break;
                    case G_SERVE:
                    default:
                        mGameState = G_IDLE;
                        break;
		}
	        break;
	    case P_LEFT:
		if(mGameState>=G_SERVE)
		    mGameState=0;
		else
		    mGameState++;
	        break;
	    case P_BOTH:
		if(mGameState>=G_SERVE)
		    mGameState=0;
		else if(mGameState==G_IDLE || mGameState==G_INIT)
		    mGameState++;
	        break;
	}
	return getMessage(mGameState);
    }

    void UserPostProcessing::setPoseActionToOut(cv::Mat& output, Array<float>& poseKeypoints)
    {
        std::string mPoseAction = "";
        std::string mMessage = "";
        std::string mMessageLine2 = "";
        int mUpperPose = 0;

        try
        {
            if(output.empty())
                error("Wrong imput cv:mat output", __LINE__, __FUNCTION__, __FILE__);
            // border relative to size
            const auto borderMargin = intRound(fastMax(output.cols, output.rows) * 0.025);
            // Used colors
            const cv::Scalar red{10, 10, 240};
            const cv::Scalar white{255, 255, 255};

            // Get pose action
            mPoseAction = "" + getPoseAction(poseKeypoints);
            // Process pose action
            mPoseAction = "" + processState(mPoseAction);

            // Get game message
            mUpperPose = getUpperPose(poseKeypoints);
            mMessage = "" + processGameState(mUpperPose);
            mMessageLine2 = "" + getMessageLine2(mGameState);
            // Draw Message on output
            putTextOnCvMat(output, mMessage,
                           {borderMargin, borderMargin}, red, false, output.cols);
            putTextOnCvMat(output, mMessageLine2,
                           {borderMargin, borderMargin*2}, red, false, output.cols);
            // Draw Action on output
            putTextOnCvMat(output, " - " + mPoseAction, 
                           {intRound(output.cols - borderMargin), borderMargin*2},
                           white, true, output.cols);
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }

    // COMPILE_TEMPLATE_DATUM(UserPostProcessing);
}

#endif // OPENPOSE_EXAMPLES_TUTORIAL_USER_POST_PROCESSING_HPP
