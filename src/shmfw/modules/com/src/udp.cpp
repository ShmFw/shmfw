/***************************************************************************
 *   Software License Agreement (BSD License)                              *  
 *   Copyright (C) 2012 by Markus Bader <markus.bader@tuwien.ac.at>        *
 *                                                                         *
 *   Redistribution and use in source and binary forms, with or without    *
 *   modification, are permitted provided that the following conditions    *
 *   are met:                                                              *
 *                                                                         *
 *   1. Redistributions of source code must retain the above copyright     *
 *      notice, this list of conditions and the following disclaimer.      *
 *   2. Redistributions in binary form must reproduce the above copyright  *
 *      notice, this list of conditions and the following disclaimer in    *
 *      the documentation and/or other materials provided with the         *
 *      distribution.                                                      *
 *   3. Neither the name of the copyright holder nor the names of its      *
 *      contributors may be used to endorse or promote products derived    *
 *      from this software without specific prior written permission.      *
 *                                                                         *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   *
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     *
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     *
 *   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE        *
 *   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  *
 *   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,  *
 *   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      *
 *   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      *
 *   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT    *
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY *
 *   WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           *
 *   POSSIBILITY OF SUCH DAMAGE.                                           *
 ***************************************************************************/
#include <shmfw/com/udp.h>

using namespace ShmFw;

UDP::UDP() {
    use_callback_ = false;
    state_ = NA;
}

UDP::~UDP() {
    if ( thread_receiverPtr_.get() != NULL ) thread_receiverPtr_->join();
}


int UDP::initReceiver ( unsigned short port, int queySize ) {
    if ( socketPtr_.get() ) {
        std::cerr << "UDP::initReceiver: class already initialized!\n";
        return 1;
    }
    try {
        socketPtr_ = SocketPtr ( new udp::socket ( ioService_, udp::endpoint ( udp::v4(), port ) ) );
        max_queue = queySize;
        bindFnc();
        state_ = RECEIVER;
    } catch ( std::exception& e ) {
        std::cerr << "Exception: " << e.what() << "\n";
        return 1;
    }
    return 0;
}

void UDP::runThread() {
    if ( thread_receiverPtr_.get() == NULL ) {
        thread_receiverPtr_ = ThreadPtr ( new boost::thread ( boost::bind ( &ShmFw::UDP::run, this ) ) );
    } else {
        std::cerr << "UDP::runThread, thread exists already!\n";
    }
}
bool UDP::isRunning() {
    if ( socketPtr_.get() ) {
        return 1;
    } else {
        return 0;
    }
}
void UDP::stopThread() {
    if ( thread_receiverPtr_.get() != NULL ) {
        ioService_.stop();
        thread_receiverPtr_->join();
        thread_receiverPtr_.reset();
    } else {
        std::cerr << "UDP::stopThread, no thread to stop!\n";
    }
}

int UDP::initTransmitter ( std::string &host, unsigned short port ) {
    if ( socketPtr_.get() ) {
        std::cerr << "UDP::initReceiver: class already initialized!\n";
        return 1;
    }
    try {
        socketPtr_ = SocketPtr ( new udp::socket ( ioService_, udp::endpoint ( udp::v4(), 0 ) ) );
        resolverPtr = ResolverPtr ( new udp::resolver ( ioService_ ) );
        queryPtr = QueryPtr ( new udp::resolver::query ( udp::v4(), host, boost::lexical_cast<std::string> ( port ) ) );
        state_ = TRANSMITTER;
    } catch ( std::exception& e ) {
        std::cerr << "Exception: " << e.what() << "\n";
        return 1;
    }
    return 0;
}

void UDP::send ( const char* data, size_t size_in_bytes ) {
    udp::resolver::iterator iterator_;
    iterator_ = resolverPtr->resolve ( *queryPtr );
    socketPtr_->send_to ( boost::asio::buffer ( data, size_in_bytes ), *iterator_ );
}


void UDP::handle_receive_from ( const boost::system::error_code& error, size_t bytes_recvd ) {
   
    if  ( !error && bytes_recvd > 0 ){
        mutex_.lock();
        std::shared_ptr<std::string> msg ( new std::string (receiveBuffer_, bytes_recvd ) );
        while ( msg_queue_.size() > max_queue ) {
            msg_queue_.pop_back();
        }
        msg_queue_.push_front ( msg );
        mutex_.unlock();
        if ( use_callback_ ) {
            callback_ ( msg );
        }
    } else {
        std::cerr << "received object does not match size : " << bytes_recvd << " bytes received!\n";
    }
    bindFnc();
}

/**
 * deques a element threadsave
 * @param msg copy of the dequed object
 * @param front on false it will deque the oldest message
 */
int UDP::deque ( std::shared_ptr<std::string> &msgPtr,  bool front ) {
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock ( mutex_ );
    if ( msg_queue_.size() > 0 ) {
        if ( front ) {
            msgPtr = msg_queue_.front();
            msg_queue_.pop_front();
        } else {
            msgPtr = msg_queue_.back();
            msg_queue_.pop_back();
        }
    }
    return msg_queue_.size();
}
