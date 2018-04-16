#ifndef OPENPOSE_EXAMPLES_TUTORIAL_USER_POST_PROCESSING_HPP
#define OPENPOSE_EXAMPLES_TUTORIAL_USER_POST_PROCESSING_HPP

#include <openpose/core/common.hpp>
#include <openpose/core/cvMatToOpInput.hpp>

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
        void setPoseActionToOut(cv::Mat& output, Array<float>& poseKeypoints);

    private:
        /**
         * Description of your variable here.
         */
        // bool mSomeVariableHere;
    };
}





// Implementation
namespace op
{
    UserPostProcessing::UserPostProcessing()
    {
        try
        {
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

    std::string getPoseAction(Array<float>& poseKeypoints) //, Array<long long> poseIds)
    {
        try
        {
            if(poseKeypoints.empty())
            {
                return "";
            }
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
        // Default pose set
        return "Standing";
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
            const cv::Scalar red{255, 0, 0};

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
