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

#ifndef SHARED_MEM_MODULE_COM_H
#define SHARED_MEM_MODULE_COM_H


#include <iostream>
#include <deque>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread/thread.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/concept_check.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <shmfw/vector.h>



namespace ShmFw {
using boost::asio::ip::udp;
///Simple class to handle UDP messages
class UDP {
    typedef boost::shared_ptr<boost::thread> ThreadPtr;
    typedef boost::shared_ptr<udp::socket> SocketPtr;
    typedef boost::shared_ptr<udp::resolver::query> QueryPtr;
    typedef boost::shared_ptr<udp::resolver> ResolverPtr;
    typedef boost::shared_ptr<std::stringstream> StringStreamPtr;
    typedef boost::shared_ptr<std::string> StringPtr;
public:
    enum State {
        NA = 0,
        RECEIVER = 1,
        TRANSMITTER = 2,
    };
    /**
     * Constructor
     * @post initReceiver or initTransmitter
     **/
    UDP();
    ///Destructor
    ~UDP();

    /**
     * Initializes the class as receiver
     * @param port
     * @param queySize
     */
    int initReceiver ( unsigned short port, int queySize = 1 );

    /**
     * Initializes the class as transmitter (sender)
     * @param host
     * @param port
     */
    int initTransmitter ( std::string &host, unsigned short port );


    /**
     * Sends a object
     * @param data
     * @param size_in_bytes
     */
	void send ( const char* data, size_t size_in_bytes );

    /**
     * starts with the receiving
     */
    void run() {
        ioService_.run();
    }

    /**
     * starts a thread to receive
     */
    void runThread();

    /**
     * stops the thread to receive
     */
    void stopThread() ;
    /**
     * returns ture if a thread is running
     */
    bool isRunning();

    /**
     * deques a element threadsave
     * @param msg copy of the dequed object
     * @param front on false it will deque the oldest message
     */
    int deque ( boost::shared_ptr<std::string> &msgPtr, bool front = true );

    /**
     * Returns the configuration send or receiver
     * @see NA, RECEIVER, TRANSMITTER
     */
    State getState() {
        return state_;
    }

    /**
     * Sets a callbackfunction which is called on incommig messages
     * @param f
     * @code udp_.setCallback(boost::bind(&CDS::Crane::Receiver::receiveCallback, this, _1));
     */
    void setCallback ( boost::function<void ( boost::shared_ptr<std::string> &msgPtr ) > f ) {
        use_callback_ = true;
        callback_ = f;
    }

private:
    inline void bindFnc() {
        try {
            socketPtr_->async_receive_from ( boost::asio::buffer ( receiveBuffer_, max_length ), sender_endpoint_, boost::bind (
                                                 &UDP::handle_receive_from, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred ) );

        } catch ( std::exception& e ) {
            std::cerr << "Exception: " << e.what() << "\n";
        }
    }
    void handle_receive_from ( const boost::system::error_code& error, size_t bytes_recvd );

private:
    static const size_t max_length = 0xFFF;
    State state_;
    boost::asio::io_service ioService_;
    SocketPtr socketPtr_;
    QueryPtr queryPtr;
    ResolverPtr resolverPtr;
    udp::endpoint sender_endpoint_;
    void *pReceiveFncPtr_;
    char receiveBuffer_[max_length];
    ThreadPtr thread_receiverPtr_;
    std::deque< boost::shared_ptr<std::string> > msg_queue_;
    unsigned int max_queue;
    boost::interprocess::interprocess_mutex mutex_; /// used to prevent concurrent access to the data
    bool use_callback_;
    boost::function<void ( boost::shared_ptr<std::string>& ) > callback_;

};
};

#endif /* SHARED_MEM_MODULE_COM_H */

