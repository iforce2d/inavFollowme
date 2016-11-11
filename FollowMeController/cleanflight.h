/*
Some functions and their supporting structs taken from Cleanflight source code. Most likely this
could be made much more succinct, but copying was quicker and easier, and ensure the right result.
*/

typedef struct gpsOrigin_s {
    bool    valid;
    float   scale;
    int32_t lat;    // Lattitude * 1e+7
    int32_t lon;    // Longitude * 1e+7
    int32_t alt;    // Altitude in centimeters (meters * 100)
} gpsOrigin_s;

/* LLH Location in NEU axis system */
typedef struct gpsLocation_s {
    int32_t lat;    // Lattitude * 1e+7
    int32_t lon;    // Longitude * 1e+7
    int32_t alt;    // Altitude in centimeters (meters * 100)
} gpsLocation_t;

// Floating point 3 vector.
typedef struct fp_vector {
    float X;
    float Y;
    float Z;
} t_fp_vector_def;

typedef union {
    float A[3];
    t_fp_vector_def V;
} t_fp_vector;

/* Geodetic functions */
typedef enum {
    GEO_ALT_ABSOLUTE,
    GEO_ALT_RELATIVE
} geoAltitudeConversionMode_e;

#define M_PIf       3.14159265358979323846f

#define DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR    1.113195f  // MagicEarthNumber from APM

// cleanflight default is to use FAST_MATH
/*
// http://lolengine.net/blog/2011/12/21/better-function-approximations
// Chebyshev http://stackoverflow.com/questions/345085/how-do-trigonometric-functions-work/345117#345117
// Thanks for ledvinap for making such accuracy possible! See: https://github.com/cleanflight/cleanflight/issues/940#issuecomment-110323384
// https://github.com/Crashpilot1000/HarakiriWebstore1/blob/master/src/mw.c#L1235
#if defined(FAST_MATH) || defined(VERY_FAST_MATH)
#if defined(VERY_FAST_MATH)
#define sinPolyCoef3 -1.666568107e-1f
#define sinPolyCoef5  8.312366210e-3f
#define sinPolyCoef7 -1.849218155e-4f
#define sinPolyCoef9  0
#else*/
#define sinPolyCoef3 -1.666665710e-1f                                          // Double: -1.666665709650470145824129400050267289858e-1
#define sinPolyCoef5  8.333017292e-3f                                          // Double:  8.333017291562218127986291618761571373087e-3
#define sinPolyCoef7 -1.980661520e-4f                                          // Double: -1.980661520135080504411629636078917643846e-4
#define sinPolyCoef9  2.600054768e-6f                                          // Double:  2.600054767890361277123254766503271638682e-6
//#endif

float sin_approx(float x)
{
    int32_t xint = x;
    if (xint < -32 || xint > 32) return 0.0f;                               // Stop here on error input (5 * 360 Deg)
    while (x >  M_PIf) x -= (2.0f * M_PIf);                                 // always wrap input angle to -PI..PI
    while (x < -M_PIf) x += (2.0f * M_PIf);
    if (x >  (0.5f * M_PIf)) x =  (0.5f * M_PIf) - (x - (0.5f * M_PIf));   // We just pick -90..+90 Degree
    else if (x < -(0.5f * M_PIf)) x = -(0.5f * M_PIf) - ((0.5f * M_PIf) + x);
    float x2 = x * x;
    return x + x * x2 * (sinPolyCoef3 + x2 * (sinPolyCoef5 + x2 * (sinPolyCoef7 + x2 * sinPolyCoef9)));
}

float cos_approx(float x)
{
    return sin_approx(x + (0.5f * M_PIf));
}

void geoConvertGeodeticToLocal(gpsOrigin_s * origin, gpsLocation_t * llh, t_fp_vector * pos, geoAltitudeConversionMode_e altConv)
{
    // Origin can only be set if GEO_ALT_ABSOLUTE to get a valid reference
    if ((!origin->valid) && (altConv == GEO_ALT_ABSOLUTE)) {
        origin->valid = true;
        origin->lat = llh->lat;
        origin->lon = llh->lon;
        origin->alt = llh->alt;
        origin->scale = constrain(cos_approx((abs(origin->lat) / 10000000.0f) * 0.0174532925f), 0.01f, 1.0f);
    }

    if (origin->valid) {
        pos->V.X = (llh->lat - origin->lat) * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR;
        pos->V.Y = (llh->lon - origin->lon) * (DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR * origin->scale);

        // If flag GEO_ALT_RELATIVE, than llh altitude is already relative to origin
        if (altConv == GEO_ALT_RELATIVE) {
            pos->V.Z = llh->alt;
        } else {
            pos->V.Z = llh->alt - origin->alt;
        }
    }
    else {
        pos->V.X = 0.0f;
        pos->V.Y = 0.0f;
        pos->V.Z = 0.0f;
    }
}

void geoConvertLocalToGeodetic(gpsOrigin_s * origin, t_fp_vector * pos, gpsLocation_t * llh)
{
    float scaleLonDown;

    if (origin->valid) {
        llh->lat = origin->lat;
        llh->lon = origin->lon;
        llh->alt = origin->alt;
        scaleLonDown = origin->scale;
    }
    else {
        llh->lat = 0;
        llh->lon = 0;
        llh->alt = 0;
        scaleLonDown = 1.0f;
    }

    llh->lat += lrintf(pos->V.X / DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR);
    llh->lon += lrintf(pos->V.Y / (DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR * scaleLonDown));
    llh->alt += lrintf(pos->V.Z);
}

