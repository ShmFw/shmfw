/*
    <one line to give the library's name and an idea of what it does.>
    Copyright (C) 2012 Markus Bader

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/

#ifndef V4R_CAM_H
#define V4R_CAM_H

#include <boost/thread/thread.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>

#include <string>
#include <vector>
#include <sys/time.h>

#define V4L2_CID_BASE_EXTCTR				0x009A0900
#define V4L2_CID_BASE_LOGITECH				V4L2_CID_BASE_EXTCTR

struct vdIn;
struct v4l2_queryctrl;
struct v4l2_control;
class V4LCam
{
public:
    static const int OK = 0;
    static const int ERROR = 1;
    static const int NA = 0;
    static const int LOAD = 1;
    static const int SAVE = 2;
    typedef int FD;
    typedef boost::shared_ptr<v4l2_control> v4l2_controlPtr;
    class ControlEntry
    {
    public:
        ControlEntry(int id);
        ~ControlEntry();
        bool valid;
        std::string varName;
        v4l2_queryctrl *queryctrl;
        int currentValue;
        int targetValue;
        std::stringstream info_msg;
        std::stringstream error_msg;
        std::string getQueryCtrlInfo() const;
    public:
        bool hasValidType() const;
        std::string pullErrorMsg() ;
        std::string pullInfoMsg() ;
        bool hasErrorMsg() const;
        bool hasInfoMsg() const;
    };
    typedef boost::shared_ptr<ControlEntry> ControlEntryPtr;
    V4LCam();
    ~V4LCam();
    bool grab();
protected:
    vdIn *pVideoIn_;
    std::string videoDevice_;
    std::string aviFilename_;
    int format_;
    int grabmethod_;
    int width_;
    int height_;
    float fps_;
    timeval timeLastFrame_;
    double durationLastFrame_;
    int camera_control_action_;
    boost::interprocess::interprocess_mutex mutexImage_;
    std::vector<ControlEntryPtr > controlEntries_;

public:
    /**
     * Initializedes a video device
     * @param videoDevice like /dev/video0
     * @return fd to the video device (integer) usabel for v4l2_ioctl calles
     **/
    FD initCamera(const std::string &videoDevice = "");

    /// v4lcontrols

    /**
     * Delivers info to a control entry
     * @param entry entry to check
     * @return error >= 1, ok = 0 details are written into entry.error_msg and entry.info_msg
     **/
    int v4lgetInfo(ControlEntryPtr entry);
    /**
     * reads a control entry and stores the value into entry.currentValue
     * @param entry entry to get
     * @return error >= 1, ok = 0 details are written into entry.error_msg and entry.info_msg
     * @pre v4lgetInfo must be called at least one befor with this control instance
     **/
    int v4lget(ControlEntryPtr entry);

    /**
     * sets a control entry using entry.targetValue
     * @param entry entry to set
     * @return error >= 1, ok = 0 details are written into entry.error_msg and entry.info_msg
     * @pre v4lgetInfo must be called at least one befor with this control instance
     **/
    int  v4lset(ControlEntryPtr entry);
    /**
     * updates a control entry using entry.targetValue
     * it also checks control min, max, stepsize
     * it also verifies the written data by reading the value again into  entry.currentValue
     * @param entry entry to update
     * @return error >= 1, ok = 0 details are written into entry.error_msg and entry.info_msg
     * @pre v4lgetInfo must be called at least one befor with this control instance
     **/
    int  v4lupdate(ControlEntryPtr entry);

    /** check if a control is backlisted
     * @param control
     * @return true if it is backlisted
     **/
    bool isBlackListed(int control);

    /**
     * detects the supporting v4l controls of the device
     * @returns controls
     **/
    const std::vector<ControlEntryPtr > &detectControlEnties();
    /**
     * saves control entries to a file
     * @return error >= 1, ok = 0 details are written into info_msg_ and entry.error_msg_
     * @see pullErrorMsg pullInfoMsg hasErrorMsg hasInfoMsg
     **/
    int save_controls(const std::string &filename);

    /**
     * loads control entries to a file
     * @return error >= 1, ok = 0 details are written into info_msg_ and entry.error_msg_
     * @see pullErrorMsg pullInfoMsg hasErrorMsg hasInfoMsg
     **/
    int load_controls(const std::string &filename);


    static char removeNonAlNum(char in);
    /**
     * @returns error messages
     **/
    std::string pullErrorMsg() ;
    /**
     * @returns info messages
     **/
    std::string pullInfoMsg() ;
    /**
     * @returns to if there are whaiting error messages
     **/
    bool hasErrorMsg() const;
    /**
     * @returns to if there are whaiting info messages
     **/
    bool hasInfoMsg() const;
protected:
    std::stringstream info_msg_;
    std::stringstream error_msg_;
};

#endif // V4R_CAM_H


