#ifndef OPENPOSE_TUTORIAL_USER_DATUM_HPP
#define OPENPOSE_TUTORIAL_USER_DATUM_HPP

#include <opencv2/core/core.hpp> // cv::Mat
#include <openpose/core/datum.hpp>
#include <openpose/core/common.hpp>

namespace op
{
    // If the user needs his own variables to be shared accross Worker classes, he can directly add them here.
    struct UserDatum : public Datum
    {
        /**
         * To keep track of the last pose action.
         * And how many frames with no pose.
         */
        std::string lastPoseAction;
        int numNoPoseFrames;

        UserDatum() {
            lastPoseAction = "";
            numNoPoseFrames = 0;
        };

        virtual ~UserDatum(){};
    };
}

#endif // OPENPOSE_TUTORIAL_USER_DATUM_HPP
