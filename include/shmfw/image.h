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

#ifndef SHARED_MEM_IMAGE_H
#define SHARED_MEM_IMAGE_H

#include <shmfw/header.h>
#include <boost/interprocess/containers/vector.hpp>

namespace ShmFw {

class Image;

/// Exdented header (not used in our case)
class SharedHeaderImage : protected SharedHeader {
    friend class boost::serialization::access;
public:
    int encoding;        /// encoding
    int width;           /// width = columns
    int height;          /// height = rows
    int depth;           /// depth  in bytes
    int channels;        /// channels  in bytes
    int widthStep;       /// Size of aligned image row in bytes.  width*channels*depth
    int pixelStep;       /// Size of aligned pixel row in bytes.  channels*depth
    /// serialize function
    void updateStepValues() {
        pixelStep =  channels * depth;
        widthStep = width * pixelStep;
    }
};

/// Class to manage a shared vectors
class Image : public Header {
public:
    enum image_encodings {
        NA = 0,
        MONO8 = 0x10, MONO16, MONO32F, MONO64F,
        RGB8 = 0x20, RGBA8, RGB16, RGBA16,
        BGR8 = 0x30, BGRA8, BGR16, BGRA16,
        YUV8 = 0x40, YUV16, YUV422
    };

    /// Local data
    struct LocalDataImg : public LocalData<unsigned char> {
        unsigned imageSize;
    };
private:
    SharedHeaderImage *header_shm;       /// exdented shared Header
    LocalDataImg  data_local;                /// local data
public:
    /** Constructor
     * @see Image::construct
     **/
    Image() {
    }
    /** Constructor
     * @param name name of the variable
     * @param shmHdl pointer to the shared memory segment handler
     * @pre the ShmPtr poitner must be created first ShmFw::createSegment
     **/
    Image ( const std::string &name, HandlerPtr &shmHdl ) {
        if ( construct ( name, shmHdl ) == ERROR ) exit ( 1 );
    }
    /** Constructor
     * @param name name of the variable
     * @param shmHdl pointer to the shared memory segment handler
     * @param height image height
     * @param width  image width
     * @param depth  size in bytes
     * @param channels  image channels
     * @param encoding  image encoding see ShmFw::Image::image_encodings
     * @param update  on true it will overwrite the exiting encoding or with an height if it still maches the size
     * @pre the ShmPtr poitner must be created first ShmFw::createSegment
     **/
    Image ( const std::string &name, HandlerPtr &shmHdl, int width, int height, int channels, int depth, int encoding = NA, bool update = false ) {
        if ( construct ( name, shmHdl, width,  height, channels, depth, encoding, update ) == ERROR ) exit ( 1 );
    }
    /**
     * @param name name of the variable
     * @param shmHdl pointer to the shared memory segment handler
     * @pre the ShmPtr poitner must be created first ShmFw::createSegment
     **/
    int construct ( const std::string &name, HandlerPtr &shmHdl ) {
#if __cplusplus > 199711L
        size_t type_hash_code = typeid ( Image ).hash_code(); 
	const char *type_name = typeid ( Image ).name();
#else
        size_t type_hash_code = 0; const char *type_name = typeid ( Image ).name();
#endif
        if ( constructHeader<SharedHeaderImage> ( name, shmHdl, type_name, type_hash_code ) == ERROR ) return ERROR;
            header_shm = ( SharedHeaderImage * ) pHeaderShm;
            if ( pHeaderShm->array_size > 0 ) {
            data_local.creator = false;
            pHeaderShm->container = ShmFw::Header::CONTAINER_IMAGE;
            updatePtr();
            } else {
                /// constructing shared data
                std::cerr << "Error the shared image: " << name << " does not exit!" << std::endl;
                exit ( 1 );
            }
        updateTimestampLocal();
        return OK;
    }
    /**
     * @param name name of the variable
     * @param shmHdl pointer to the shared memory segment handler
     * @param height image height
     * @param width  image width
     * @param depth  size in bytes
     * @param channels  image channel
     * @param encoding  image encoding see ShmFw::Image::image_encodings
     * @param update  on true it will overwrite the exiting encoding or with an height if it still maches the size
     * @pre the ShmPtr poitner must be created first
     * @see ShmFw::createSegment
     **/
    int construct ( const std::string &name, HandlerPtr &shmHdl, int width, int height, int channels, int depth, int encoding, bool update = false ) {
#if __cplusplus > 199711L
        size_t type_hash_code = typeid ( Image ).hash_code(); 
	const char *type_name = typeid ( Image ).name();
#else
        size_t type_hash_code = 0; 
	const char *type_name = typeid ( Image ).name();
#endif
        if ( constructHeader<SharedHeaderImage> ( name, shmHdl, type_name, type_hash_code ) == ERROR ) return ERROR;
            header_shm = ( SharedHeaderImage * ) pHeaderShm;
            if ( pHeaderShm->array_size > 0 ) {
            data_local.creator = false;
            data_local.ptr = ( unsigned char * ) pHeaderShm->ptr.get();
                int pixelStep = channels * depth;
                int widthStep = width * pixelStep;
                unsigned int size = height * widthStep;
                if ( pHeaderShm->array_size != size ) {
                    if ( update ) {
                        try {
                            ScopedLock myLock ( pHeaderShm->mutex );
                            headerLoc.pShmHdl->getShm()->deallocate ( pHeaderShm->ptr.get() );
                            pHeaderShm->array_size = 0;
                        } catch ( ... ) {
                            std::cerr << "Error when deallocate shared data" << std::endl;
                            exit ( 1 );
                        }
                    } else {
                        std::cerr << "Exiting shared image differs size: " << data_local.imageSize << " != " << pHeaderShm->array_size << std::endl;
                    }

                }
            }
        if ( pHeaderShm->array_size == 0 ) {
        /// constructing shared data
        try {
            ScopedLock myLock ( pHeaderShm->mutex );
                header_shm->encoding = encoding;
                header_shm->width = width, header_shm->height = height, header_shm->channels = channels, header_shm->depth = depth;
                header_shm->updateStepValues();
                pHeaderShm->array_size = header_shm->widthStep * header_shm->height;
                if ( headerLoc.pShmHdl->getShm()->get_size() > pHeaderShm->array_size ) {
                    pHeaderShm->ptr = headerLoc.pShmHdl->getShm()->allocate ( pHeaderShm->array_size );
                    data_local.creator = true;
                    updatePtr();
                } else {
                    std::cerr << "Image for shared memory to big: " << headerLoc.pShmHdl->getShm()->get_size() << " < " << pHeaderShm->array_size << std::endl;
                    exit ( 1 );
                }
            } catch ( ... ) {
                std::cerr << "Error when constructing shared data" << std::endl;
                exit ( 1 );
            }
        }
        updateTimestampLocal();
        return OK;
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a reference to the shared header
     * @warning do not use this fnc, it is only for serialization
     * @return ref to shared data
     **/
    SharedHeaderImage &shared_header() {
        return *header_shm;
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a reference to the shared header
     * @warning do not use this fnc, it is only for serialization
     * @return ref to shared data
     **/
    const SharedHeaderImage &shared_header() const {
        return *header_shm;
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * Returns pointer to the shared raw image data
     * @return pointer to the first pixel
     **/
    unsigned char *get ( int x, int y ) const {
        int offset = y * header_shm->widthStep + x * header_shm->pixelStep;
        return data_local.ptr + offset;
    };
    /** UNSAVE!! (user have to lock and to update timestamp)
     * sets a 1 channel image entry but does not check if it is a 1 channel image
     * @param x  x position (column)
     * @param y  y position (row)
     * @param c0 1st channel entry
     **/
    template <class T>
    void set ( int x, int y, const T &c0 ) {
        T *p = ( T * ) get ( x, y );
        *p = c0;
    };
    /** UNSAVE!! (user have to lock and to update timestamp)
     * sets a 2 channel image entry but does not check if it is a 2 channel image
     * @param x  x position (column)
     * @param y  y position (row)
     * @param c0 1st channel entry
     * @param c1 2nd channel entry
     * @param c2 3rd channel entry
     **/
    template <class T>
    void set ( int x, int y, const T &c0, const T &c1 ) {
        T *p = ( T * ) get ( x, y );
        *p++ = c0;
        *p = c1;
    };
    /** UNSAVE!! (user have to lock and to update timestamp)
     * sets a 3 channel image entry but does not check if it is a 3 channel image
     * @param x  x position (column)
     * @param y  y position (row)
     * @param c0 1st channel entry
     * @param c1 2nd channel entry
     * @param c2 3rd channel entry
     **/
    template <class T>
    void set ( int x, int y, const T &c0, const T &c1, const T &c2 ) {
        T *p = ( T * ) get ( x, y );
        *p++ = c0;
        *p++ = c1;
        *p = c2;
    };
    /** UNSAVE!! (user have to lock and to update timestamp)
     * sets a 4 channel image entry but does not check if it is a 4 channel image
     * @param x  x position (column)
     * @param y  y position (row)
     * @param c0 1st channel entry
     * @param c1 2nd channel entry
     * @param c2 3rd channel entry
     * @param c3 4th channel entry
     **/
    template <class T>
    void set ( int x, int y, const T &c0, const T &c1, const T &c2, const T &c3 ) {
        T *p = ( T * ) get ( x, y );
        *p++ = c0;
        *p++ = c1;
        *p++ = c2;
        *p = c3;
    };
    /** UNSAVE!! (user have to lock and to update timestamp)
     * Updates the local data pointer if the shared data changed
     * @return true if the pointer changed
     **/
    bool updatePtr() {
        if ( data_local.imageSize != pHeaderShm->array_size ) {
            data_local.ptr = ( unsigned char * ) pHeaderShm->ptr.get();
            data_local.imageSize = pHeaderShm->array_size;
            return true;
        } else {
            return false;
        }
    };
    /** UNSAVE!! (user have to lock and to update timestamp)
     * Returns pointer to the shared raw image data
     * @return pointer to the first pixel
     **/
    unsigned char *data() const {
        return data_local.ptr;
    };
    /** UNSAVE!! (user have to lock and to update timestamp)
     * Returns the shared image with
     * @return with
     **/
    int width() const {
        return header_shm->width;
    };
    /** UNSAVE!! (user have to lock and to update timestamp)
     * Returns the shared image height
     * @return height
     **/
    int height() const {
        return header_shm->height;
    };
    /** UNSAVE!! (user have to lock and to update timestamp)
     * Returns the shared image channels
     * @return channels
     **/
    int channels() const {
        return header_shm->channels;
    };
    /** UNSAVE!! (user have to lock and to update timestamp)
     * Returns the shared image depth
     * @return depth in bytes
     **/
    int depth() const {
        return header_shm->depth;
    };
    /** UNSAVE!! (user have to lock and to update timestamp)
     * Returns the shared image widthStep depth*channels*width
     * @return widthStep in bytes
     **/
    int widthStep() const {
        return header_shm->widthStep;
    };
    /** UNSAVE!! (user have to lock and to update timestamp)
     * Returns the shared image encoding
     * @see ShmFw::Image::image_encodings
     * @return encoding
     **/
    int encoding() const {
        return header_shm->encoding;
    };
    /** UNSAVE!! (user have to lock and to update timestamp)
     * Returns the shared image encoding as string
     * @see ShmFw::Image::image_encodings
     * @return encoding
     **/
    static std::string encoding ( int id ) {
        if ( id == NA ) return "NA";
        if ( id == MONO8 ) return "MONO8";
        if ( id == MONO16 ) return "MONO16";
        if ( id == RGB8 ) return "RGB8";
        if ( id == RGBA8 ) return "RGBA8";
        if ( id == RGB16 ) return "RGB16";
        if ( id == RGBA16 ) return "RGBA16";
        if ( id == BGR8 ) return "BGR8";
        if ( id == BGRA8 ) return "BGRA8";
        if ( id == BGR16 ) return "BGR16";
        if ( id == BGRA16 ) return "BGRA16";
        if ( id == YUV8 ) return "YUV8";
        if ( id == YUV16 ) return "YUV16";
        if ( id == BGRA16 ) return "BGRA16";
        return "UNKOWN";
    };
    /** UNSAVE!! (user have to lock and to update timestamp)
     * Returns the shared image pixelStep  depth*channels
     * @return pixelStep in bytes
     **/
    int pixelStep() const {
        return header_shm->pixelStep;
    };
    /** UNSAVE!! (user have to lock and to update timestamp)
     * Returns the shared image rowSize
     * @return rowSize
     **/
    unsigned int imageSize() const {
        return pHeaderShm->array_size;
    };
    virtual void destroy() const {
        headerLoc.pShmHdl->getShm()->destroy_ptr ( data_local.ptr );
        Header::destroy();
    };
    /** UNSAVE!! (user have to lock and to update timestamp)
     * Returns a human readable string to show the context
     * @return string
     **/
    virtual std::string human_readable() const {
        std::stringstream ss;
        ss << name() << " is a image " << width() << "x" << height() << ", ";
        ss << channels() << " channels, " << depth() << " depth, ";
        ss << imageSize()  << " bytes = " << ( ( float ) imageSize() ) / 1024.0 / 1024.0 << "MB";
        ss << encoding()  << " encoding = " <<  encoding ( encoding() ) ;
        return ss.str();
    };
    /** UNSAVE!! (user have to lock and to update timestamp)
     **/
    void copyToShm ( const void *src ) {
        memcpy ( pHeaderShm->ptr.get(), src, pHeaderShm->array_size );
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     **/
    void copyfromShm ( void *des ) const {
        memcpy ( des, pHeaderShm->ptr.get(), pHeaderShm->array_size );
    }
};
};
#endif //SHARED_MEM_IMAGE_H
