/*
    <one line to give the library's name and an idea of what it does.>
    Copyright (C) 2012  Markus Bader

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
#include <iomanip>
#include <v4lcam/v4l_cam_defaults.h>
#include <v4lcam/v4l_cam.h>
#include <boost/algorithm/string.hpp>

extern "C" {
#include <libv4l2.h>
#include "v4lcam/v4l2uvc.h"
#include "v4lcam/color.h"
#include <linux/videodev2.h>
}

/* Fixed point arithmetic */
#define FIXED Sint32
#define FIXED_BITS 16
#define TO_FIXED(X) (((Sint32)(X))<<(FIXED_BITS))
#define FROM_FIXED(X) (((Sint32)(X))>>(FIXED_BITS))


#define INCPANTILT 64 // 1°


#include <boost/interprocess/sync/scoped_lock.hpp>
typedef boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> Lock;

static const char version[] = "VERSION";

char V4LCam::removeNonAlNum(char in)
{
    if(in == ' ') return '_';
    if(!isalnum(in)) return '_';
    return in;
}

V4LCam::~V4LCam()
{
    if(pVideoIn_) {
        close_v4l2(pVideoIn_);
        free(pVideoIn_);
    }
    freeLut();
}

V4LCam::V4LCam()
    : pVideoIn_(NULL)
    , videoDevice_(DEFAULT_VIDEODEVICE)
    , aviFilename_(DEFAULT_AVIFILENAME)
    , format_(DEFAULT_FORMAT)
    , grabmethod_(DEFAULT_GRABMETHODE)
    , width_(DEFAULT_WIDTH)
    , height_(DEFAULT_HEIGHT)
    , fps_(DEFAULT_FPS)
    , camera_control_action_(NA)
{
    mutexImage_.unlock();
}

V4LCam::FD V4LCam::initCamera(const std::string &videoDevice)
{
    if(!videoDevice.empty()) {
        videoDevice_ = videoDevice;
    }
    pVideoIn_ = (struct vdIn *) calloc(1, sizeof(struct vdIn));
    if(init_videoIn(pVideoIn_, (char *) videoDevice_.c_str(), width_, height_, fps_, format_, grabmethod_, (char *) aviFilename_.c_str()) < 0)
        exit(1);
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    if(uvcGrab(pVideoIn_) < 0) {
        printf("Error grabbing first image\n");
        return 0;
    }
    initLut();
    gettimeofday(&timeLastFrame_, NULL);
    return pVideoIn_->fd;
}


bool V4LCam::grab()
{

    boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    Lock myLock(mutexImage_);
    if(uvcGrab(pVideoIn_) < 0) {
        printf("Error grabbing\n");
        return false;
    }
    if(pVideoIn_->signalquit != 1) {
        printf("videoIn->signalquit\n");
        return false;
    }
    timeval now;
    gettimeofday(&now, NULL);
    durationLastFrame_ = (now.tv_sec - timeLastFrame_.tv_sec) * 1000.0;      // sec to ms
    durationLastFrame_ += (now.tv_usec - timeLastFrame_.tv_usec) / 1000.0;   // us to ms
    timeLastFrame_ = now;

    return true;
}


int V4LCam::v4lgetInfo(ControlEntryPtr entry)
{
    if(entry->valid == false) {
        entry->error_msg << "v4lgetInfo not valid\n";
        return ERROR;
    }
    if((v4l2_ioctl(pVideoIn_->fd, VIDIOC_QUERYCTRL, entry->queryctrl)) < 0) {
        entry->valid = false;
        entry->error_msg << "v4l2_ioctl querycontrol error\n";
        return ERROR;
    } else if(entry->queryctrl->flags & V4L2_CTRL_FLAG_DISABLED) {
        entry->valid = false;
        entry->error_msg << "control disabled\n";
        return ERROR;
    } else if(entry->hasValidType() == false) {
        entry->valid = false;
        entry->error_msg << "unsupported type\n";
        return ERROR;
    }
    entry->varName = std::string((const char *) entry->queryctrl->name);
    boost::algorithm::to_lower(entry->varName);

    std::transform(entry->varName.begin(), entry->varName.end(), entry->varName.begin(), V4LCam::removeNonAlNum);
    boost::algorithm::trim_left_if(entry->varName, boost::algorithm::is_any_of("_"));
    boost::algorithm::trim_right_if(entry->varName, boost::algorithm::is_any_of("_"));
    boost::algorithm::replace_all(entry->varName, "___", "_");
    boost::algorithm::replace_all(entry->varName, "__", "_");

    return OK;
}


int V4LCam::v4lget(ControlEntryPtr entry)
{
    if(entry->valid == false) {
        entry->error_msg << "v4lget not valid\n";
        return ERROR;
    }
    struct v4l2_control control;
    control.id = entry->queryctrl->id;

    if(v4l2_ioctl(pVideoIn_->fd, VIDIOC_G_CTRL, &control) < 0) {
        entry->error_msg <<  "v4l2_ioctl get control error\n";
        return ERROR;
    }
    entry->currentValue = control.value;
    boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    entry->info_msg << "current = " << entry->currentValue << "\n";
    return OK;
}

int V4LCam::v4lset(ControlEntryPtr entry)
{
    if(entry->valid == false) {
        entry->error_msg << "v4lset not valid\n";
        return ERROR;
    }
    struct v4l2_control control;
    control.id = entry->queryctrl->id;
    control.value = entry->targetValue;
    if(v4l2_ioctl(pVideoIn_->fd, VIDIOC_S_CTRL, &control) < 0) {
        entry->error_msg <<  "v4l2_ioctl set( " << control.value << ") control error\n";
        return ERROR;
    }
    boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    entry->currentValue = entry->targetValue;
    entry->info_msg << "current = " << entry->currentValue << "\n";
    return OK;
}
int V4LCam::v4lupdate(ControlEntryPtr entry)
{
    if(entry->currentValue == entry->targetValue) {
        return OK;
    }
    if(entry->valid == false) {
        entry->currentValue = entry->targetValue;
        entry->error_msg << "v4lupdate not valid\n";
        return ERROR;
    }

    if(entry->targetValue % entry->queryctrl->step) {
        entry->info_msg << "target value " << entry->targetValue << "between steps " <<  entry->queryctrl->step;
        entry->targetValue = (entry->targetValue / entry->queryctrl->step) * entry->queryctrl->step;
        entry->info_msg << "new target value = " << entry->targetValue << std::endl;
    }
    if(entry->targetValue > entry->queryctrl->maximum) {
        entry->info_msg << "clipping taget value = " << entry->targetValue << " to maximum = " << entry->queryctrl->maximum << "\n";
        entry->targetValue = entry->queryctrl->maximum;
    }
    if(entry->targetValue < entry->queryctrl->minimum) {
        entry->info_msg << "clipping taget value = " << entry->targetValue << " to minimum = " << entry->queryctrl->minimum << "\n";
        entry->targetValue = entry->queryctrl->minimum;
    }

    struct v4l2_control control;
    control.id = entry->queryctrl->id;
    control.value = entry->targetValue;
    if(v4l2_ioctl(pVideoIn_->fd, VIDIOC_S_CTRL, &control) < 0) {
        entry->error_msg <<  "v4l2_ioctl set( " << control.value << ") control error\n";
        return ERROR;
    }
    if(entry->queryctrl->flags & V4L2_CTRL_FLAG_WRITE_ONLY) {
        entry->info_msg << "entry is write only\n";
        entry->currentValue = entry->targetValue;
    } else {
        if(v4l2_ioctl(pVideoIn_->fd, VIDIOC_G_CTRL, &control) < 0) {
            entry->error_msg <<  "v4l2_ioctl get control error, to verify\n";
            return ERROR;
        }
        entry->currentValue = control.value;
        if(control.value != entry->targetValue) {
            entry->error_msg <<  "v4l2_ioctl set and get are different. -> ";
            entry->error_msg <<  entry->targetValue << " != " << control.value << std::endl;
            return ERROR;
        }
    }
    entry->info_msg << "current = " << entry->currentValue << "\n";
    return OK;
}

V4LCam::ControlEntry::ControlEntry(int id)
    : valid(true)
    , varName("NA")
    , queryctrl(new v4l2_queryctrl)
    , currentValue(-1)
    , targetValue(-1)
{
    queryctrl->id = id;
}
V4LCam::ControlEntry::~ControlEntry()
{
    delete queryctrl;
}
bool V4LCam::ControlEntry::hasValidType() const
{
    if(queryctrl->type == V4L2_CTRL_TYPE_INTEGER) return true;
    if(queryctrl->type == V4L2_CTRL_TYPE_BOOLEAN)  return true;
    if(queryctrl->type == V4L2_CTRL_TYPE_MENU) return true;
    if(queryctrl->type == V4L2_CTRL_TYPE_BUTTON) return true;
    if(queryctrl->type == V4L2_CTRL_TYPE_INTEGER64) return true;
    if(queryctrl->type == V4L2_CTRL_TYPE_CTRL_CLASS) return true;
    if(queryctrl->type == V4L2_CTRL_TYPE_STRING) return true;
    if(queryctrl->type == V4L2_CTRL_TYPE_BITMASK) return true;
    if(queryctrl->type == V4L2_CTRL_TYPE_BITMASK) return true;
    return false;
}
std::string V4LCam::ControlEntry::getQueryCtrlInfo() const
{
    std::stringstream ss;

    ss  << std::setw(9) << std::hex << std::showbase << queryctrl->id << " = " << varName;
    if(valid == false) return ss.str();
    ss << " >> " << queryctrl->name << std::endl;
    ss << std::dec << "current = " << currentValue << ", target = "  <<  targetValue;
    ss << ", min = " << queryctrl->minimum << ", max = " << queryctrl->maximum;
    ss << ", default = " << queryctrl->default_value << ", step = " << queryctrl->step << std::endl;
    ss << "flags =";
    if(queryctrl->flags & V4L2_CTRL_FLAG_DISABLED) ss  << " disabled";
    if(queryctrl->flags & V4L2_CTRL_FLAG_GRABBED)  ss << " grabbed";
    if(queryctrl->flags & V4L2_CTRL_FLAG_READ_ONLY)   ss << " read only";
    if(queryctrl->flags & V4L2_CTRL_FLAG_UPDATE)  ss << " update";
    if(queryctrl->flags & V4L2_CTRL_FLAG_INACTIVE) ss << " inactive";
    if(queryctrl->flags & V4L2_CTRL_FLAG_SLIDER) ss << " slider";
    if(queryctrl->flags & V4L2_CTRL_FLAG_WRITE_ONLY) ss << " write only";
    if(queryctrl->flags & V4L2_CTRL_FLAG_VOLATILE) ss << " volatile";
    if(queryctrl->flags == 0) ss << " empty";
    ss << "; type =";
    if(queryctrl->type == V4L2_CTRL_TYPE_INTEGER) ss << " integer";
    else if(queryctrl->type == V4L2_CTRL_TYPE_BOOLEAN) ss << "boolean";
    else if(queryctrl->type == V4L2_CTRL_TYPE_MENU)  ss << "menu";
    else if(queryctrl->type == V4L2_CTRL_TYPE_BUTTON)  ss << "button";
    else if(queryctrl->type == V4L2_CTRL_TYPE_INTEGER64)  ss << "integer64";
    else if(queryctrl->type == V4L2_CTRL_TYPE_CTRL_CLASS)  ss << "ctrl_class";
    else if(queryctrl->type == V4L2_CTRL_TYPE_STRING)  ss << "string";
    else if(queryctrl->type == V4L2_CTRL_TYPE_BITMASK)  ss << "bitmask";
    else ss << "unsupported";
    return ss.str();
}

std::string V4LCam::ControlEntry::pullErrorMsg()
{
    std::string str = error_msg.str();
    error_msg.str(std::string());
    return str;
}
std::string V4LCam::ControlEntry::pullInfoMsg()
{
    std::string str = info_msg.str();
    info_msg.str(std::string());
    return str;
}
bool V4LCam::ControlEntry::hasErrorMsg() const
{
    return !error_msg.str().empty();
};
bool V4LCam::ControlEntry::hasInfoMsg() const
{
    return !info_msg.str().empty();
};

std::string V4LCam::pullErrorMsg()
{
    std::string str = error_msg_.str();
    error_msg_.str(std::string());
    return str;
}
std::string V4LCam::pullInfoMsg()
{
    std::string str = info_msg_.str();
    info_msg_.str(std::string());
    return str;
}
bool V4LCam::hasErrorMsg() const
{
    return !error_msg_.str().empty();
};
bool V4LCam::hasInfoMsg() const
{
    return !info_msg_.str().empty();
};
const std::vector<V4LCam::ControlEntryPtr > &V4LCam::detectControlEnties()
{
    v4l2_queryctrl queryctrl;

    controlEntries_.clear();

    /* Try the extended control API first */
#ifdef V4L2_CTRL_FLAG_NEXT_CTRL  // ref --> v4l3upc
    queryctrl.id = V4L2_CTRL_FLAG_NEXT_CTRL;
    if(0 == v4l2_ioctl(pVideoIn_->fd, VIDIOC_QUERYCTRL, &queryctrl)) {
        do {
            boost::this_thread::sleep(boost::posix_time::milliseconds(10));
            controlEntries_.push_back(ControlEntryPtr(new ControlEntry(queryctrl.id)));
            queryctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
        } while(0 == v4l2_ioctl(pVideoIn_->fd, VIDIOC_QUERYCTRL, &queryctrl));
    } else
#endif
    {
        /* Fall back on the standard API */
        /* Check all the standard controls */
        for(int i = V4L2_CID_BASE; i < V4L2_CID_LASTP1; i++) {
            controlEntries_.push_back(ControlEntryPtr(new ControlEntry(i)));
        }
        /* Check any custom controls */
        for(queryctrl.id = V4L2_CID_PRIVATE_BASE; (v4l2_ioctl(pVideoIn_->fd, VIDIOC_QUERYCTRL, &queryctrl) == 0) ; queryctrl.id++) {
            controlEntries_.push_back(ControlEntryPtr(new ControlEntry(queryctrl.id)));
            boost::this_thread::sleep(boost::posix_time::milliseconds(10));
        }
    }
    for(unsigned int i = 0; i < controlEntries_.size(); i++) {
        v4lgetInfo(controlEntries_[i]);
    }
    return controlEntries_;
}


int V4LCam::save_controls(const std::string &filename)
{
    struct v4l2_queryctrl queryctrl;
    struct v4l2_control   control_s;
    FILE *configfile;
    memset(&queryctrl, 0, sizeof(queryctrl));
    memset(&control_s, 0, sizeof(control_s));
    configfile = fopen(filename.c_str(), "w");
    if(configfile == NULL) {
        error_msg_ << "saving configfile: " << filename << " failed" << std::endl;
    } else {
        fprintf(configfile, "id         value      # luvcview control settings configuration file\n");
        for(std::vector<V4LCam::ControlEntryPtr>::const_iterator it = controlEntries_.begin(); it != controlEntries_.end(); it++) {
            queryctrl.id = (*it)->queryctrl->id;
            if(0 == ioctl(pVideoIn_->fd, VIDIOC_QUERYCTRL, &queryctrl)) {
                if(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
                    continue;
                control_s.id = queryctrl.id;
                v4l2_ioctl(pVideoIn_->fd, VIDIOC_G_CTRL, &control_s);
                boost::this_thread::sleep(boost::posix_time::milliseconds(10));
                fprintf(configfile, "%-10d %-10d # name:%-32s type:%d min:%-5d max:%-5d step:%-5d def:%d\n",
                        queryctrl.id, control_s.value, queryctrl.name, queryctrl.type, queryctrl.minimum,
                        queryctrl.maximum, queryctrl.step, queryctrl.default_value);
                printf("%-10d %-10d # name:%-32s type:%d min:%-5d max:%-5d step:%-5d def:%d\n",
                       queryctrl.id, control_s.value, queryctrl.name, queryctrl.type, queryctrl.minimum,
                       queryctrl.maximum, queryctrl.step, queryctrl.default_value);
                boost::this_thread::sleep(boost::posix_time::milliseconds(10));
            }
        }
        fflush(configfile);
        fclose(configfile);
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        info_msg_ << "saved configfile: " << filename << std::endl;
    }
    return OK;
}

int V4LCam::load_controls(const std::string &filename)//struct vdIn *vd)
{
    struct v4l2_control   control;
    FILE *configfile;
    memset(&control, 0, sizeof(control));
    configfile = fopen(filename.c_str(), "r");
    if(configfile == NULL) {
        error_msg_ << "configfile: " << filename << " open failed" << std::endl;
    } else {
        info_msg_ << "loading controls from luvcview.cfg\n";
        char buffer[0xFFF];
        fgets(buffer, sizeof(buffer), configfile);
        while(NULL != fgets(buffer, sizeof(buffer), configfile)) {
            sscanf(buffer, "%i%i", &control.id, &control.value);
            if(v4l2_ioctl(pVideoIn_->fd, VIDIOC_S_CTRL, &control))
                info_msg_ << "ERROR id: " << control.id << " val: " <<  control.value << std::endl;
            else
                info_msg_ << "OK    id: " << control.id << " val: " <<  control.value << std::endl;
            boost::this_thread::sleep(boost::posix_time::milliseconds(20));
        }
        fclose(configfile);
        info_msg_ << "loaded configfile: " << filename << std::endl;
    }
    return OK;
}

