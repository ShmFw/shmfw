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

#ifndef SHARED_MEM_DEQUE_H
#define SHARED_MEM_DEQUE_H

#include <shmfw/header.h>
#include <boost/interprocess/containers/deque.hpp>
#include <shmfw/serialization/interprocess_deque.hpp>

namespace ShmFw {

/// Class to manage a shared vectors
template<typename T>
class Deque : public Header {

public:
    typedef bi::allocator<T, SegmentManager> Allocator;
    typedef bi::deque<T, Allocator > DequeShm;
    typedef typename Allocator::size_type size_type;

protected:
public:

    /** Default constructor
     * @post Deque::construct
     **/
    Deque() {
    }

    /** Constructor
     * @param name name of the variable
     * @param shmHdl pointer to the shared memory segment
     * @pre the ShmPtr poitner must be created first
     * @see ShmFw::createSegment
     **/
    Deque ( const std::string &name, HandlerPtr &shmHdl ) {
        if ( construct ( name, shmHdl ) == ERROR ) exit ( 1 );
    }

    /**
     * @param name name of the variable
     * @param shmHdl pointer to the shared memory segment
     * @pre the ShmPtr poitner must be created first
     * @see ShmFw::createSegment
     **/
    int construct ( const std::string &name, HandlerPtr &shmHdl ) {
#if __cplusplus > 199711L
        size_t type_hash_code = typeid ( Deque<T> ).hash_code();
        const char *type_name = typeid ( Deque<T> ).name();
#else
        size_t type_hash_code = 0;
        const char *type_name = typeid ( Deque<T> ).name();
#endif
        if ( constructHeader ( name, shmHdl, type_name, type_hash_code ) == ERROR ) return ERROR;
        if ( headerLoc.creator ) {
            /// constructing shared data
            try {
                ScopedLock myLock ( pHeaderShm->mutex );
                pHeaderShm->container = ShmFw::Header::CONTAINER_DEQUE;
                Allocator a ( headerLoc.pShmHdl->getShm()->get_segment_manager() );
                pHeaderShm->data = headerLoc.pShmHdl->getShm()->construct< DequeShm > ( bi::anonymous_instance ) ( a );
            } catch ( ... ) {
                std::cerr << "Error when constructing shared data" << std::endl;
                return ERROR;
            }
        }
        return OK;
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a pointer to the shared object
     * @return ref to shared data
     **/
    DequeShm *ptr() {
        return (DequeShm*) pHeaderShm->data.get();
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a pointer to the shared object
     * @return ref to shared data
     **/
    const DequeShm *ptr() const {
        return (DequeShm*) pHeaderShm->data.get();
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a reference to the shared vector
     * @return ref to shared data
     **/
    const DequeShm &ref() const {
        return *ptr();
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a reference to the shared vector
     * @return ref to shared data
     **/
    DequeShm &ref() {
        return *ptr();
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a reference to the shared vector
     * @return ref to shared data
     **/
    const DequeShm &operator() () const {
        return ref();
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a reference to the shared vector
     * @return ref to shared data
     **/
    DequeShm &operator() () {
        return ref();
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a reference to the shared vector object by index
     * @return ref to shared data
     **/
    const T &operator [] ( size_type n ) const {
        return ref() [n];
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a reference to the shared vector object by index
     * @return ref to shared data
     **/
    T &operator [] ( size_type n ) {
        return ref() [n];
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a reference to the shared vector object by index
     * @return ref to shared data
     **/
    const T &at ( size_type n ) const {
        return ptr()->at ( n );
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a reference to the shared vector object by index
     * @return ref to shared data
     **/
    T &at ( size_type n ) {
        return ptr()->at ( n );
    }
    /** SAVE ACCESS :-) (the function will to the lock and the timstamp stuff)
     * copies data to the shared variable and updated the timestamps and locks the variable while accessing
     * @param src source vector
     **/
    template<typename T1>
    void set ( const T1 &src ) {
        set ( src, true );
    }
    /** CAN BE SAVE ACCESS :-) (the function will to the lock and the timstamp stuff)
     * copies data to the shared variable
     * on locking is true it will also updated the timestamps and locks the variable while accessing
     * @param src source vector
     * @param locking on true it will lock and trigger a change otherwise the function is UNSAVE!!
     **/
    template<typename T1>
    void set ( const T1 &src, bool locking ) {
        if ( locking ) lock();
        ptr()->resize ( src.size() );
        for ( size_t i = 0; i < src.size(); i++ ) {
            ref() [i] = src[i];
        }
        if ( locking ) unlock();
        if ( locking ) itHasChanged();
    }
    /** SAVE ACCESS :-) (the function will to the lock and the timstamp stuff)
     * copies data to the shared vector index and updated the timestamps and locks the variable while accessing
     * @param src source vector
     * @param n index
     **/
    void set ( const T &src, size_type n ) {
        set ( src, n, true );
    }
    /** CAN BE SAVE ACCESS :-) (the function will to the lock and the timstamp stuff)
     * copies a data element to the shared variable
     * on locking is true it will also updated the timestamps and locks the variable while accessing
     * @param src source vector
     * @param n index
     * @param locking on true it will lock and trigger a change otherwise the function is UNSAVE!!
     **/
    void set ( const T &src, size_type n, bool locking ) {
        if ( locking ) lock();
        ref() [n] = src;
        if ( locking ) unlock();
        if ( locking ) itHasChanged();
    }
    /** SAVE ACCESS :-) (the function will to the lock and the timstamp stuff)
     *  copies data form the shared variable into a local varaiable and sets local timestamps and locks the variable while accessing
     * @param des
     **/
    void get ( std::vector<T> &des ) {
        get ( des, true );
    }
    /** CAN BE SAVE ACCESS :-) (the function will to the lock and the timstamp stuff)
     *  copies data form the shared variable into a local varaiable
     *  on locking is true it will also updated the timestamps and locks the variable while accessing
     * @param des
     * @param locking on true it will lock and trigger a change otherwise the function is UNSAVE!!
     **/
    void get ( std::vector<T> &des, bool locking ) {
        if ( locking ) lock();
        des.resize ( ptr()->size() );
        for ( unsigned int i = 0; i < des.size(); i++ ) {
            des[i] = ref() [i];
        }
        if ( locking ) unlock();
        if ( locking ) updateTimestampLocal();
    }
    /** SAVE ACCESS :-) (the function will to the lock and the timstamp stuff)
     *  copies data element n form the shared vetor index into a local varaiable and will also updated the timestamps and locks the variable while accessing
     * @param des
     * @param n index
     **/
    void get ( T &des, size_type n ) {
        get ( des, n, true );
    }
    /** SAVE ACCESS :-) (the function will to the lock and the timstamp stuff)
     *  copies data element n form the shared vetor index into a local varaiable
     *  on locking is true it will also updated the timestamps and locks the variable while accessing
     * @param des
     * @param n index
     * @param locking on true it will lock and trigger a change otherwise the function is UNSAVE!!
     **/
    void get ( T &des, size_type n, bool locking ) {
        if ( locking ) lock();
        des = ref() [n];
        if ( locking ) unlock();
        if ( locking ) updateTimestampLocal();
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * Returns a human readable string to show the context
     * @return string
     **/
    virtual std::string human_readable() const {
        std::stringstream ss;
        ss << name() << " = [";
        for ( size_t i = 0; i < ptr()->size(); i++ ) {
            ss << ( ( i == 0 ) ? " " : ", " ) << std::setw ( 10 ) << ptr()->at ( i );
        }
        ss << "]";
        return ss.str();
    };
    /** UNSAVE!! (user have to lock and to update timestamp)
     * @return vetor size
     **/
    size_t size() const {
        return ptr()->size();
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * @return Returns true if the vector contains no elements
     **/
    bool empty() const {
        return ptr()->empty();
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * Inserts or erases elements at the end such that the size becomes n. New elements are default constructed.
     **/
    void resize ( size_t n ) {
        ptr()->resize ( n );
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * Inserts or erases elements at the end such that the size becomes n. New elements are default constructed.
     **/
    void resize ( size_t n, const T& v ) {
        ptr()->resize ( n, v );
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * Inserts a copy of x at the end of the vector.
     **/
    void push_front ( const T &src ) {
        ptr()->push_front ( src );
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     *  Removes the first element in the deque container, effectively reducing its size by one.
     **/
    void pop_front() {
        ptr()->pop_front();
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     *  Returns a reference to the first element in the deque container.
     **/
    const T &front() const {
        return ptr()->front();
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     *  Returns a reference to the first element in the deque container.
     **/
    T &front() {
        return ptr()->front();
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * Inserts a copy of x at the end of the vector.
     **/
    void push_back ( const T &src ) {
        ptr()->push_back ( src );
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * Removes the last element in the deque container, effectively reducing the container size by one.
     **/
    void pop_back() {
        ptr()->pop_back();
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     *  Returns a reference to the last element in the container.
     **/
    const T & back() const {
        return ptr()->back();
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     *  Returns a reference to the last element in the container.
     **/
    T & back() {
        return ptr()->back();
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     *  Erases all the elements of the deque..
     **/
    void clear() {
        return ptr()->clear();
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * destroies the shared memory
     * @ToDo
     **/
    virtual void destroy() const {
        // headerLoc.pShmHdl->getShm()->destroy_ptr(data_local.ptr);
        // Header::destroy();
        std::cerr << "vector::destroy() -> kown to have problem!" << std::endl;
    };
    /** UNSAVE!! (user have to lock and to update timestamp)
     *  @param o vector for comparison
     **/
    template<typename T1>
    bool operator == (const T1 &o ) const {
        if ( size() != o.size() ) return false;
	//return (memcmp(&ref()[0], &o[0], size()) == 0);
        for ( size_t i = 0; i < size(); i++ ) if ( ref() [i] != o[i] ) return false;
        return true;
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     *  @param o vector for comparison
     **/
    template<typename T1>
    bool operator != (const T1 &o ) const {
        if ( size() != o.size() ) return true;
	//return (memcmp(&ref()[0], &o[0], size()) != 0);
        for ( size_t i = 0; i < size(); i++ ) if ( ref() [i] != o[i] ) return true;
        return false;
    }
};

class DequeStr : public ShmFw::Deque<ShmFw::CharString> {
public:
    /** Default constructor
     * @post Deque::construct
     **/
    DequeStr()
        : ShmFw::Deque<ShmFw::CharString>() {
    }

    /** Constructor
     * @param name name of the variable
     * @param shmHdl pointer to the shared memory segment
     * @pre the ShmPtr poitner must be created first
     * @see ShmFw::createSegment
     **/
    DequeStr ( const std::string &name, HandlerPtr &shmHdl )
        : ShmFw::Deque<ShmFw::CharString> ( name, shmHdl ) {
    }

    /** UNSAVE!! (user have to lock and to update timestamp)
     * Inserts a copy of x at the end of the vector.
     **/
    void push_back ( const std::string &str ) {
        CharString shmStr = headerLoc.pShmHdl->createString ( str );
        ptr()->push_back ( shmStr );
    }
};

};
#endif //SHARED_MEM_DEQUE_H
