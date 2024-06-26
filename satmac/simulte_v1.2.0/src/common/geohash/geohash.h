/*
 *Copyright (c) 2013-2014, yinqiwen <yinqiwen@gmail.com>
 *All rights reserved.
 *
 *Redistribution and use in source and binary forms, with or without
 *modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of Redis nor the names of its contributors may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 *THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 *BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 *THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef GEOHASH_H_
#define GEOHASH_H_

#include "mbr-common.h"
#include <string>
#ifdef LINUX_KERNEL

#include <linux/types.h>

#define LAT_RANGE_MIN 39.74732
#define LAT_RANGE_MAX 40.15929
#define LON_RANGE_MIN 116.16677
#define LON_RANGE_MAX 116.73407

    typedef enum
    {
        GEOHASH_NORTH = 0,
        GEOHASH_EAST,
        GEOHASH_WEST,
        GEOHASH_SOUTH,
        GEOHASH_SOUTH_WEST,
        GEOHASH_SOUTH_EAST,
        GEOHASH_NORT_WEST,
        GEOHASH_NORT_EAST
    } GeoDirection;

    typedef struct
    {
            uint64_t bits;
            uint8_t step;
    } GeoHashBits;

        typedef struct
    {
            double max;
            double min;
    } GeoHashRange;

    typedef struct
    {
            GeoHashBits north;
            GeoHashBits east;
            GeoHashBits west;
            GeoHashBits south;
            GeoHashBits north_east;
            GeoHashBits south_east;
            GeoHashBits north_west;
            GeoHashBits south_west;
    } GeoHashNeighbors;


    int geohash_get_neighbors(GeoHashBits hash, GeoHashNeighbors* neighbors);
    int geohash_get_neighbor(GeoHashBits hash, GeoDirection direction, GeoHashBits* neighbor);
    int geohash_get_neighbors_in_set(GeoHashSetCoordinate * geohashset, u64 center_geohash, int geohash_step);

    GeoHashBits geohash_next_leftbottom(GeoHashBits bits);
    GeoHashBits geohash_next_rightbottom(GeoHashBits bits);
    GeoHashBits geohash_next_lefttop(GeoHashBits bits);
    GeoHashBits geohash_next_righttop(GeoHashBits bits);

    geohash_fast_encode(GeoHashRange lat_range, GeoHashRange lon_range, double latitude, double longitude, uint8_t step, GeoHashBits* hash);

#else

#include <stdint.h>

#define unlikely(a) a

#define LAT_RANGE_MIN -90
#define LAT_RANGE_MAX 90
#define LON_RANGE_MIN -180
#define LON_RANGE_MAX 180

//#define LAT_RANGE_MIN 40.644513
//#define LAT_RANGE_MAX 40.922891
//#define LON_RANGE_MIN -74.144698
//#define LON_RANGE_MAX -73.65602


#if defined(__cplusplus)
extern "C"
{
#endif

    typedef enum
    {
        GEOHASH_NORTH = 0,
        GEOHASH_EAST,
        GEOHASH_WEST,
        GEOHASH_SOUTH,
        GEOHASH_SOUTH_WEST,
        GEOHASH_SOUTH_EAST,
        GEOHASH_NORT_WEST,
        GEOHASH_NORT_EAST
    } GeoDirection;

    typedef struct
    {
            uint64_t bits;
            uint8_t step;
    } GeoHashBits;

    typedef struct
    {
            double max;
            double min;
    } GeoHashRange;

    typedef struct
    {
            GeoHashBits hash;
            GeoHashRange latitude;
            GeoHashRange longitude;
    } GeoHashArea;

    typedef struct
    {
            GeoHashBits north;
            GeoHashBits east;
            GeoHashBits west;
            GeoHashBits south;
            GeoHashBits north_east;
            GeoHashBits south_east;
            GeoHashBits north_west;
            GeoHashBits south_west;
    } GeoHashNeighbors;

    /*
     * 0:success
     * -1:failed
     */
    int geohash_encode(GeoHashRange lat_range, GeoHashRange lon_range, double latitude, double longitude, uint8_t step, GeoHashBits* hash);
    int geohash_decode(GeoHashRange lat_range, GeoHashRange lon_range, GeoHashBits hash, GeoHashArea* area);

    /*
     * Fast encode/decode version, more magic in implementation.
     */
    int geohash_fast_encode(GeoHashRange lat_range, GeoHashRange lon_range, double latitude, double longitude, uint8_t step, GeoHashBits* hash);
    int geohash_fast_decode(GeoHashRange lat_range, GeoHashRange lon_range, GeoHashBits hash, GeoHashArea* area);

    int geohash_get_neighbors(GeoHashBits hash, GeoHashNeighbors* neighbors);
    int geohash_get_neighbor(GeoHashBits hash, GeoDirection direction, GeoHashBits* neighbor);
    int geohash_get_neighbors_in_set(GeoHashSetCoordinate * geohashset, uint64_t center_geohash, int geohash_step);
    int geohash_is_geohash_in_set(uint64_t geohash, GeoHashSetCoordinate geohashset);

    GeoHashBits geohash_next_leftbottom(GeoHashBits bits);
    GeoHashBits geohash_next_rightbottom(GeoHashBits bits);
    GeoHashBits geohash_next_lefttop(GeoHashBits bits);
    GeoHashBits geohash_next_righttop(GeoHashBits bits);

    std::string dec2bin(int64_t num);


#if defined(__cplusplus)
}
#endif
#endif /* LINUX_KERNEL */
#endif /* GEOHASH_H_ */
