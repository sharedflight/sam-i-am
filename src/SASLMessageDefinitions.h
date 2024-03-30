
#define MSG_ADD_DATAREF 0x01000000           //  Add dataref to DRE message

#define SF_MSG_SEND 0x8034201;
#define SF_MSG_RCV 0x8034202;

struct SASL_MSG_IntArrayData {
    size_t mSize ;
    int * mData ;
};
struct SASL_MSG_FloatArrayData {
    size_t mSize ;
    float * mData ;
};
struct SASL_MSG_StringData {
    size_t mSize ;
    const char * mData ;
};
