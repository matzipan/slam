#ifndef UTILS_H
#define UTILS_H

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#include <string>
#include <vector>

////////////////////////////////////////////////////////////////////////////////
// time functions
////////////////////////////////////////////////////////////////////////////////
uint64_t getMillisecondsTime(void);

////////////////////////////////////////////////////////////////////////////////
// ASSERT macro
////////////////////////////////////////////////////////////////////////////////
#ifdef RTK_DEBUG

#define ASSERT(f) \
        do { \
            if (!f ) { \
                fprintf(stderr, "ERROR (FILE: %s, LINE: %d, FUNC: %s)\n", \
                    __FILE__, __LINE__, __FUNCTION__); \
                exit(1); \
            } \
        } while (0); \


#define ASSERT2(f, s) \
        do { \
            if (!f ) { \
                fprintf(stderr, "ERROR: %s (FILE: %s, LINE: %d, FUNC: %s)\n", \
                    s, __FILE__, __LINE__, __FUNCTION__); \
                exit(1); \
            } \
        } while (0); \

#else
#define ASSERT(f)
#define ASSERT2(f, s)
#endif

////////////////////////////////////////////////////////////////////////////////
// string functions
////////////////////////////////////////////////////////////////////////////////
typedef std::vector<std::string> StringArray;

// split given string by delims
StringArray split_text(const std::string &intext, const std::string &delims);

std::string trim(const std::string &s);

////////////////////////////////////////////////////////////////////////////////
// arguments functions
////////////////////////////////////////////////////////////////////////////////
void save_arguments(int argc, char *argv[], std::string fname);

StringArray path_splitext(const std::string &fname);

/*****************************************************************************\
\*****************************************************************************/
enum CVariantType {
    VT_INT,                 // integer value
    VT_FLOAT,               // float value
    VT_DOUBLE,              // double value
    VT_STRING,              // string value
    VT_POINTER,             // pointer value
    VT_BIN,                 // binary value
};

union CVariantUnion {
    int iVal;           // integer value
    float fVal;           // float value
    double dVal;           // double value
    char *sVal;          // string value
    void *pVal;          // pointer value
    void *bVal;          // binary value
};

class CVariant {
public:
    // set value
    void set(int v);

    void set(float v);

    void set(double v);

    void set(char *v);

    void set(const char *v);

    void set(void *v);

    // get/convert type
    int to_i(void);

    float to_f(void);

    double to_d(void);

    char *to_s(char *buf);

    char *to_s(void);

    void *to_p(void);

    // get length
    int size(void);

    // to/from stream data
    int stream_len(void);

    int to_stream(int *len, uint8_t *buf);

    int from_stream(int len, uint8_t *buf);

    // assignment operator
    CVariant &operator=(const CVariant &o);

public:
    CVariant() { _init(); }

    virtual ~CVariant() { _release(); }

private:
    void _init(void);

    void _release(void);

protected:
    CVariantType t;
    CVariantUnion d;

    char *buf;
};

/*****************************************************************************\
\*****************************************************************************/
class CParamArray {
public:
    CParamArray() { _init(); }

    virtual ~CParamArray() { _release(1); }

    // get value (if exist)
    int i(const std::string &n, int &v);

    int f(const std::string &n, float &v);

    int d(const std::string &n, double &v);

    int s(const std::string &n, std::string &v);

    int p(const std::string &n, void **v);

    // get value
    int i(const std::string &n);

    float f(const std::string &n);

    double d(const std::string &n);

    std::string s(const std::string &n);

    void *p(const std::string &n);

    // get key exist
    int key_exist(const std::string &n);

    // set value
    int set_i(const std::string &n, int v);

    int set_f(const std::string &n, float v);

    int set_d(const std::string &n, double v);

    int set_s(const std::string &n, std::string v);

    int set_p(const std::string &n, void *v);

    // if not exist, set value
    // if exist, get value
    int setget_i(const std::string &n, int &v);

    int setget_f(const std::string &n, float &v);

    int setget_d(const std::string &n, double &v);

    int setget_s(const std::string &n, std::string &v);

    int setget_p(const std::string &n, void **v);

    // parse arguments
    int set_args(int argc, char *argv[]);

    // load / save
    virtual int load(const std::string &f);

    virtual int save(const std::string &f);

    // to/from stream data
    int stream_len(void);

    int to_stream(int *len, uint8_t *buf);

    virtual int from_stream(int len, uint8_t *buf);

    // parse item
    virtual int parse(void);

    // push/pop settings
    int push(void);

    int pop(void);

    // print parameters
    void print(void);

    // clear all items
    void clear(void);

    // assignment operator
    CParamArray &operator=(const CParamArray &o);

protected:
    std::vector<std::string> na;     // name array
    std::vector<CVariant *> va;     // value array
    std::vector<uint8_t *> sa;     // stack array

    int _init(void);

    int _release(int s);
};

/*****************************************************************************\
\*****************************************************************************/
class CArgs {
public:
    CArgs() { _init(); }

    virtual ~CArgs() { _release(); }

    // set arguments
    int set_args(int argc, char *argv[]);

    // save arguments to file
    int save(std::string fname);

protected:
    std::vector<std::string> na;     // argument array

    int _init(void);

    int _release(void);
};

#endif // __UTILS_H__
