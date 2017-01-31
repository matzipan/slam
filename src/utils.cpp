#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <cxxabi.h>

#include <sys/time.h>

#include <complex.h>

#include <vector>
#include <algorithm>

#include "utils.h"

using namespace std;

uint64_t getMillisecondsTime(void) {
    struct timeval tm_val;
    uint64_t v;
    int ret;

    ret = gettimeofday(&tm_val, NULL);

    v = tm_val.tv_sec * 1000 + tm_val.tv_usec / 1000;
    return v;
}

////////////////////////////////////////////////////////////////////////////////
// string functions
////////////////////////////////////////////////////////////////////////////////

// split given string by delims
StringArray split_text(const string &intext, const string &delims) {
    StringArray r;
    string::size_type begIdx, endIdx;
    string s;

    begIdx = intext.find_first_not_of(delims);

    while (begIdx != string::npos) {
        // search end of the word
        endIdx = intext.find_first_of(delims, begIdx);
        if (endIdx == string::npos)
            endIdx = intext.length();

        // get the sub string
        s = intext.substr(begIdx, endIdx - begIdx);
        r.push_back(s);

        // find next begin position
        begIdx = intext.find_first_not_of(delims, endIdx);
    }

    return r;
}

string trim(const string &s) {
    string delims = " \t\n\r",
            r;
    string::size_type i, j;

    i = s.find_first_not_of(delims);
    j = s.find_last_not_of(delims);

    if (i == string::npos) {
        r = "";
        return r;
    }

    if (j == string::npos) {
        r = "";
        return r;
    }

    r = s.substr(i, j - i + 1);
    return r;
}

////////////////////////////////////////////////////////////////////////////////
// arguments functions
////////////////////////////////////////////////////////////////////////////////

void save_arguments(int argc, char *argv[], string fname) {
    string fn;
    FILE *fp;
    int i;
    tm *now;
    time_t t;
    char str_time[200];


    fn = fname + "_args.txt";
    fp = fopen(fn.c_str(), "a+"); ASSERT(fp);

    // get current time
    time(&t);
    now = localtime(&t);
    strftime(str_time, 200, "%Y-%m-%d %H:%M:%S", now);

    fprintf(fp, "--------------- %s ---------------\n", str_time);

    for (i = 0; i < argc; i++)
        fprintf(fp, "%s ", argv[i]);

    fprintf(fp, "\n\n");

    fclose(fp);
}

////////////////////////////////////////////////////////////////////////////////
// file & path functions
////////////////////////////////////////////////////////////////////////////////

StringArray path_split(const string &fname) {
    size_t found = -1;
    StringArray r;

    r.clear();

    /* find / or \ */
    found = fname.find_last_of("/\\");

    if (found == string::npos) {
        r.push_back("");
        r.push_back(fname);
        return r;
    }

    // folder
    r.push_back(fname.substr(0, found));
    // file
    r.push_back(fname.substr(found + 1));

    return r;
}

StringArray path_splitext(const string &fname) {
    size_t found;
    StringArray r;

    r.clear();

    // find .
    found = fname.find_last_of(".");
    if (found == string::npos) {
        r.push_back(fname);
        r.push_back("");
        return r;
    }

    // filename
    r.push_back(fname.substr(0, found));
    // extname
    r.push_back(fname.substr(found));

    return r;
}

std::string path_join(const std::string &p1, const std::string &p2) {
    string p;
    int l;

    p = p1;

    l = p.size();
    if (p[l - 1] == '/' || p[l - 1] == '\\')
        p = p.substr(0, l - 1);

    p = p + "/" + p2;
    return p;
}

#define PARAM_ARRAY_ITEM_MAXLEN     4096

/*****************************************************************************\
\*****************************************************************************/
void CVariant::set(int v) {
    if (t == VT_STRING) delete d.sVal;

    t = VT_INT;
    d.iVal = v;
}

void CVariant::set(float v) {
    if (t == VT_STRING) delete d.sVal;

    t = VT_FLOAT;
    d.fVal = v;
}

void CVariant::set(double v) {
    if (t == VT_STRING) delete d.sVal;

    t = VT_DOUBLE;
    d.dVal = v;
}

void CVariant::set(char *v) {
    int n;

    if (t == VT_STRING) delete d.sVal;

    t = VT_STRING;
    n = strlen(v);
    d.sVal = new char[n + 1];
    strcpy(d.sVal, v);
}

void CVariant::set(const char *v) {
    set((char *) v);
}

void CVariant::set(void *v) {
    if (t == VT_STRING) delete d.sVal;

    t = VT_POINTER;
    d.pVal = v;
}

int CVariant::to_i(void) {
    switch (t) {
        case VT_INT:
            return d.iVal;
            break;

        case VT_FLOAT:
            return (int) (d.fVal);
            break;

        case VT_DOUBLE:
            return (int) (d.dVal);
            break;

        case VT_STRING:
            return atoi(d.sVal);
            break;

        case VT_POINTER:
            return 0;
            break;
    }

    return 0;
}

float CVariant::to_f(void) {
    switch (t) {
        case VT_INT:
            return (float) (d.iVal);
            break;

        case VT_FLOAT:
            return d.fVal;
            break;

        case VT_DOUBLE:
            return (float) (d.dVal);
            break;

        case VT_STRING:
            return (float) (atof(d.sVal));
            break;

        case VT_POINTER:
            return 0;
            break;
    }

    return 0;
}

double CVariant::to_d(void) {
    switch (t) {
        case VT_INT:
            return (double) (d.iVal);
            break;

        case VT_FLOAT:
            return (double) (d.fVal);
            break;

        case VT_DOUBLE:
            return d.dVal;
            break;

        case VT_STRING:
            return atof(d.sVal);
            break;

        case VT_POINTER:
            return 0;
            break;
    }

    return 0;
}

char *CVariant::to_s(char *buf) {
    buf[0] = 0;

    switch (t) {
        case VT_INT:
            sprintf(buf, "%d", d.iVal);
            break;

        case VT_FLOAT:
            sprintf(buf, "%g", d.fVal);
            break;

        case VT_DOUBLE:
            sprintf(buf, "%g", d.dVal);
            break;

        case VT_STRING:
            return d.sVal;
            break;

        case VT_POINTER:
            // FIXME: change to u64
            sprintf(buf, "%lx", (uint64_t) d.pVal);
            break;
    }

    return buf;
}

char *CVariant::to_s(void) {
    // FIXME: use a fixed length
    if (buf == NULL) {
        buf = new char[PARAM_ARRAY_ITEM_MAXLEN];
    }

    return to_s(buf);
}

void *CVariant::to_p(void) {
    if (t == VT_POINTER)
        return d.pVal;
    else
        return NULL;
}

int CVariant::size(void) {
    switch (t) {
        case VT_INT:
            return sizeof(int);
            break;

        case VT_FLOAT:
            return sizeof(float);
            break;

        case VT_DOUBLE:
            return sizeof(double);
            break;

        case VT_STRING:
            return strlen(d.sVal) + 1;
            break;

        case VT_POINTER:
            return sizeof(void *);
            break;
    }

    return 0;
}

int CVariant::stream_len(void) {
    int l1, l2;

    l1 = sizeof(CVariantType);
    l2 = size();
    return (l1 + l2);
}

/**
 *  to stream data
 *
 *  Parameters:
 *      \param[out]     len         data length
 *      \param[out]     buf         data buff
 *  Return Value:
 *      0               successfule
 */
int CVariant::to_stream(int *len, uint8_t *buf) {
    int i, l1, l2;

    l1 = sizeof(CVariantType);
    l2 = size();
    *len = l1 + l2;

    // copy data type field
    i = 0;
    memcpy(buf + i, &t, sizeof(int));
    i += l1;

    // copy data
    switch (t) {
        case VT_INT:
            memcpy(buf + i, &(d.iVal), sizeof(int));
            break;

        case VT_FLOAT:
            memcpy(buf + i, &(d.fVal), sizeof(float));
            break;

        case VT_DOUBLE:
            memcpy(buf + i, &(d.dVal), sizeof(double));
            break;

        case VT_STRING:
            memcpy(buf + i, d.sVal, l2);
            break;

        case VT_POINTER:
            memcpy(buf + i, &(d.pVal), l2);
            break;
    }

    return 0;
}

int CVariant::from_stream(int len, uint8_t *buf) {
    int i, l1, l2;

    l1 = sizeof(CVariantType);
    l2 = len - l1;

    // copy data type field
    i = 0;
    memcpy(&t, buf + i, sizeof(int));
    i += l1;

    // copy data
    switch (t) {
        case VT_INT:
            memcpy(&(d.iVal), buf + i, sizeof(int));
            break;

        case VT_FLOAT:
            memcpy(&(d.fVal), buf + i, sizeof(float));
            break;

        case VT_DOUBLE:
            memcpy(&(d.dVal), buf + i, sizeof(double));
            break;

        case VT_STRING:
            if (d.sVal != NULL) delete d.sVal;
            d.sVal = new char[l2 + 1];
            memcpy(d.sVal, buf + i, l2);
            break;

        case VT_POINTER:
            memcpy(&(d.pVal), buf + i, l2);
            break;
    }

    return 0;
}

CVariant &CVariant::operator=(const CVariant &o) {
    if (this == &o) return *this;

    if (o.t == VT_STRING) {
        int l;
        l = strlen(o.d.sVal) + 1;

        if (t == VT_STRING) delete d.sVal;
        d.sVal = new char[l];

        strcpy(d.sVal, o.d.sVal);
        t = o.t;
    } else {
        t = o.t;
        d = o.d;
    }

    return *this;
}

void CVariant::_init(void) {
    t = VT_INT;
    memset(&d, 0, sizeof(CVariantUnion));

    buf = NULL;
}

void CVariant::_release(void) {
    if (t == VT_STRING) {
        delete d.sVal;
        t = VT_INT;
        d.iVal = 0;
    }

    if (buf != NULL) {
        delete buf;
        buf = NULL;
    }
}

/*****************************************************************************\
\*****************************************************************************/

int CParamArray::load(const string &f) {
    FILE *fp = NULL;
    char *buf;
    StringArray sa;
    string _b;

    string _n;
    CVariant *_v;

    // open parameter array file
    fp = fopen(f.c_str(), "rt");
    if (fp == NULL) {
        //dbg_pe("Failed to open file: %s\n", f.c_str());
        return -1;
    }

    // clear old data
    na.clear();
    va.clear();

    // alloc string buffer for reading file
    buf = (char *) malloc(PARAM_ARRAY_ITEM_MAXLEN);

    while (!feof(fp)) {
        // read a line
        if (NULL == fgets(buf, PARAM_ARRAY_ITEM_MAXLEN, fp))
            break;

        // remove blank & CR
        _b = trim(buf);

        if (_b.size() < 1)
            continue;

        // skip comment
        if (_b[0] == '#' || _b[0] == ':')
            continue;

        // FIXME: if current line have more than one of "="
        //        then it will be failed
        sa = split_text(_b, "=");

        if (sa.size() >= 2) {
            _n = trim(sa[0]);

            _v = new CVariant;
            _v->set(trim(sa[1]).c_str());

            na.push_back(_n);
            va.push_back(_v);
        }
    }

    // free file & buf
    free(buf);
    fclose(fp);

    // parse items
    parse();

    return 0;
}

int CParamArray::save(const string &f) {
    return 0;
}

int CParamArray::i(const string &n, int &v) {
    int i, l;

    l = na.size();
    for (i = 0; i < l; i++) {
        if (n == na[i]) {
            try {
                v = va[i]->to_i();
            } catch (...) {
                return -1;
            }

            return 0;
        }
    }

    return -1;
}

int CParamArray::i(const string &n) {
    int i, l;

    l = na.size();
    for (i = 0; i < l; i++) {
        if (n == na[i]) {
            int _v;

            try {
                _v = va[i]->to_i();
            } catch (...) {
                ASSERT2(0, "CParmArray::i >> cann't conver value");
            }

            return _v;
        }
    }

    ASSERT2(0, "CParmArray::i >> cann't conver value");
    return 0;
}


int CParamArray::f(const string &n, float &v) {
    int i, l;

    l = na.size();
    for (i = 0; i < l; i++) {
        if (n == na[i]) {
            try {
                v = va[i]->to_f();
            } catch (...) {
                return -1;
            }

            return 0;
        }
    }

    return -1;
}

float CParamArray::f(const string &n) {
    int i, l;

    l = na.size();
    for (i = 0; i < l; i++) {
        if (n == na[i]) {
            float _v;

            try {
                _v = va[i]->to_f();
            } catch (...) {
                ASSERT2(0, "CParmArray::f >> cann't conver value");
            }

            return _v;
        }
    }

    ASSERT2(0, "CParmArray::f >> cann't get value");
    return 0.0;
}


int CParamArray::d(const string &n, double &v) {
    int i, l;

    l = na.size();
    for (i = 0; i < l; i++) {
        if (n == na[i]) {
            try {
                v = va[i]->to_d();
            } catch (...) {
                return -1;
            }

            return 0;
        }
    }

    return -1;
}

double CParamArray::d(const string &n) {
    int i, l;

    l = na.size();
    for (i = 0; i < l; i++) {
        if (n == na[i]) {
            double _v;

            try {
                _v = va[i]->to_d();
            } catch (...) {
                ASSERT2(0, "CParmArray::d >> cann't conver value");
            }

            return _v;
        }
    }

    ASSERT2(0, "CParmArray::d >> cann't get value");
    return 0.0;
}

int CParamArray::s(const string &n, string &v) {
    int i, l;
    string s;

    l = na.size();
    for (i = 0; i < l; i++) {
        if (n == na[i]) {
            l = va[i]->size();
            s = va[i]->to_s();

            if (s[0] == '\"' && s[l - 1] == '\"') {
                v = s.substr(1, l - 2);
            } else {
                v = s;
            }

            return 0;
        }
    }

    return -1;
}

string CParamArray::s(const string &n) {
    int i, l;
    string s, s2;

    l = na.size();
    for (i = 0; i < l; i++) {
        if (n == na[i]) {

            l = va[i]->size();
            s = va[i]->to_s();

            if (s[0] == '\"' && s[l - 1] == '\"') {
                s2 = s.substr(1, l - 2);
            } else {
                s2 = s;
            }

            return s2;
        }
    }

    ASSERT2(0, "CParmArray::s >> cann't get value");
    return "";
}

int CParamArray::p(const string &n, void **v) {
    int i, l;

    l = na.size();
    for (i = 0; i < l; i++) {
        if (n == na[i]) {
            *v = va[i]->to_p();

            return 0;
        }
    }

    return -1;
}

void *CParamArray::p(const string &n) {
    int i, l;

    l = na.size();
    for (i = 0; i < l; i++) {
        if (n == na[i]) {
            return va[i]->to_p();
        }
    }

    return NULL;
}


int CParamArray::key_exist(const string &n) {
    int i, l;

    // find given key exist
    l = na.size();
    for (i = 0; i < l; i++) {
        if (n == na[i]) {
            return 1;
        }
    }

    return 0;
}

int CParamArray::set_i(const string &n, int v) {
    int i, l;
    string _n;
    CVariant *_v;

    // find given item exist
    l = na.size();
    for (i = 0; i < l; i++) {
        if (n == na[i]) {
            va[i]->set(v);
            return 0;
        }
    }

    // insert new item
    _n = n;
    _v = new CVariant();
    _v->set(v);
    na.push_back(_n);
    va.push_back(_v);

    return 0;
}

int CParamArray::set_f(const string &n, float v) {
    int i, l;
    string _n;
    CVariant *_v;

    // find given item exist
    l = na.size();
    for (i = 0; i < l; i++) {
        if (n == na[i]) {
            va[i]->set(v);
            return 0;
        }
    }

    // insert new item
    _n = n;
    _v = new CVariant();
    _v->set(v);
    na.push_back(_n);
    va.push_back(_v);

    return 0;
}

int CParamArray::set_d(const string &n, double v) {
    int i, l;
    string _n;
    CVariant *_v;

    // find given item exist
    l = na.size();
    for (i = 0; i < l; i++) {
        if (n == na[i]) {
            va[i]->set(v);
            return 0;
        }
    }

    // insert new item
    _n = n;
    _v = new CVariant();
    _v->set(v);
    na.push_back(_n);
    va.push_back(_v);

    return 0;
}

int CParamArray::set_s(const string &n, string v) {
    int i, l;
    string _n;
    CVariant *_v;

    // find item exist
    l = na.size();
    for (i = 0; i < l; i++) {
        if (n == na[i]) {
            va[i]->set(v.c_str());
            return 0;
        }
    }

    // insert new item
    _n = n;
    _v = new CVariant();
    _v->set(v.c_str());
    na.push_back(_n);
    va.push_back(_v);

    return 0;
}

int CParamArray::set_p(const string &n, void *v) {
    int i, l;
    string _n;
    CVariant *_v;

    // find item exist
    l = na.size();
    for (i = 0; i < l; i++) {
        if (n == na[i]) {
            va[i]->set(v);
            return 0;
        }
    }

    // insert new item
    _n = n;
    _v = new CVariant();
    _v->set(v);
    na.push_back(_n);
    va.push_back(_v);

    return 0;
}

int CParamArray::setget_i(const string &n, int &v) {
    int i, l;
    string _n;
    CVariant *_v;

    // find item exist
    l = na.size();
    for (i = 0; i < l; i++) {
        // get value
        if (n == na[i]) {
            v = va[i]->to_i();
            return 0;
        }
    }

    // set value
    _n = n;
    _v = new CVariant();
    _v->set(v);
    na.push_back(_n);
    va.push_back(_v);

    return 0;
}

int CParamArray::setget_f(const string &n, float &v) {
    int i, l;
    string _n;
    CVariant *_v;

    // find item exist
    l = na.size();
    for (i = 0; i < l; i++) {
        // get value
        if (n == na[i]) {
            v = va[i]->to_f();
            return 0;
        }
    }

    // set value
    _n = n;
    _v = new CVariant();
    _v->set(v);
    na.push_back(_n);
    va.push_back(_v);

    return 0;
}

int CParamArray::setget_d(const string &n, double &v) {
    int i, l;
    string _n;
    CVariant *_v;

    // find item exist
    l = na.size();
    for (i = 0; i < l; i++) {
        // get value
        if (n == na[i]) {
            v = va[i]->to_d();
            return 0;
        }
    }

    // set value
    _n = n;
    _v = new CVariant();
    _v->set(v);
    na.push_back(_n);
    va.push_back(_v);

    return 0;
}

int CParamArray::setget_s(const string &n, string &v) {
    int i, l;
    string _n;
    CVariant *_v;

    // find item exist
    l = na.size();
    for (i = 0; i < l; i++) {
        // get value
        if (n == na[i]) {
            v = va[i]->to_s();
            return 0;
        }
    }

    // set value
    _n = n;
    _v = new CVariant();
    _v->set(v.c_str());
    na.push_back(_n);
    va.push_back(_v);

    return 0;
}

int CParamArray::setget_p(const string &n, void **v) {
    int i, l;
    string _n;
    CVariant *_v;

    // find item exist
    l = na.size();
    for (i = 0; i < l; i++) {
        // get value
        if (n == na[i]) {
            *v = va[i]->to_p();
            return 0;
        }
    }

    // set value
    _n = n;
    _v = new CVariant();
    _v->set(*v);
    na.push_back(_n);
    va.push_back(_v);

    return 0;
}

int CParamArray::set_args(int argc, char *argv[]) {
    int i;
    char *p;
    string v;

    for (i = 1; i < argc; i++) {
        if (argv[i][0] == '-') {
            p = argv[i] + 1;
            v = argv[++i];
            set_s(p, v);
        }
    }

    return 0;
}

int CParamArray::stream_len(void) {
    int n, i;
    int l1, l2, l_all;

    // get item number
    n = na.size();

    // stream length + item number + each item's length
    l_all = sizeof(int) + sizeof(int) + sizeof(int) * n;

    // get each item's length
    for (i = 0; i < n; i++) {
        l1 = strlen(na[i].c_str()) + 1;
        l2 = va[i]->stream_len();

        l_all += sizeof(int) + l1 + l2;
    }

    return l_all;
}

/**
 *  To stream data buf
 *
 *  Parameters:
 *      \param[out]     len     data length
 *      \param[out]     buf     data stream
 *  Return Value:
 *      0               success
 */
int CParamArray::to_stream(int *len, uint8_t *buf) {
    int n, i;
    int l1, l2, l_item, p;
    int *arr_len;

    // get item number
    n = na.size();

    // alloc length array
    arr_len = new int[n];

    // get each item's length
    for (i = 0; i < n; i++) {
        l1 = strlen(na[i].c_str()) + 1;
        l2 = va[i]->stream_len();

        arr_len[i] = sizeof(int) + l1 + l2;
    }

    // generate stream
    p = 0;

    // stream total length
    p += sizeof(int);

    // item number
    memcpy(buf + p, &n, sizeof(int));
    p += sizeof(int);

    // each item's length
    for (i = 0; i < n; i++) {
        memcpy(buf + p, &(arr_len[i]), sizeof(int));
        p += sizeof(int);
    }

    // for each item
    for (i = 0; i < n; i++) {
        // name length
        l1 = strlen(na[i].c_str()) + 1;
        memcpy(buf + p, &l1, sizeof(int));
        p += sizeof(int);

        // name
        memcpy(buf + p, na[i].c_str(), l1);
        p += l1;

        // value
        va[i]->to_stream(&l_item, buf + p);
        p += l_item;
    }

    // set total length
    memcpy(buf, &p, sizeof(int));

    // return length
    *len = p;

    delete arr_len;

    return 0;
}

int CParamArray::from_stream(int len, uint8_t *buf) {
    int n, i;
    int l1, l2, p;
    char *str_name;
    int pi;
    int *arr_len;
    CVariant *v;

    // name string
    str_name = new char[PARAM_ARRAY_ITEM_MAXLEN];

    // clear old data
    clear();

    // stream position index
    p = 0;

    // stream total length
    p += sizeof(int);

    // get item number
    memcpy(&n, buf + p, sizeof(int));
    p += sizeof(int);

    // alloc length array
    arr_len = new int[n];

    // get each item's length
    for (i = 0; i < n; i++) {
        memcpy(&pi, buf + p, sizeof(int));
        arr_len[i] = pi;
        p += sizeof(int);
    }

    // for each item
    for (i = 0; i < n; i++) {
        // name length
        memcpy(&pi, buf + p, sizeof(int));
        l1 = pi;
        p += sizeof(int);

        // name
        memcpy(str_name, buf + p, l1);
        p += l1;

        // value
        l2 = arr_len[i] - sizeof(int) - l1;
        v = new CVariant();
        v->from_stream(l2, buf + p);
        p += l2;

        // add name/value to array
        na.push_back(str_name);
        va.push_back(v);
    }

    // parse items
    parse();

    // free temp variables
    delete str_name;
    delete arr_len;

    return 0;
}

int CParamArray::parse(void) {
    return 0;
}

int CParamArray::push(void) {
    int buf_len;
    uint8_t *buf;

    // to steam
    buf_len = stream_len();
    buf = new uint8_t[buf_len];
    to_stream(&buf_len, buf);

    // push to stack
    sa.push_back(buf);

    return 0;
}

int CParamArray::pop(void) {
    int buf_len;
    uint8_t *buf;

    if (sa.size() > 0) {
        buf = sa.back();

        memcpy(&buf_len, buf, sizeof(int));
        from_stream(buf_len, buf);
        delete buf;

        sa.pop_back();
    }

    return 0;
}

void CParamArray::print(void) {
    int i, l, n_l, max_l, max_l_def;
    char str_fmt[300];

    // item number
    l = na.size();

    // determin max name length
    max_l_def = 10;
    max_l = max_l_def;
    for (i = 0; i < l; i++) {
        n_l = strlen(na[i].c_str());
        if (n_l > max_l) max_l = n_l;
    }

    if (max_l > max_l_def) max_l += 2;

    // generate format string
    sprintf(str_fmt, "%%%ds = %%s\n", max_l + 2);

    // print
    printf("-------------------- Parameters -------------------------\n");

    for (i = 0; i < l; i++)
        printf(str_fmt, na[i].c_str(), va[i]->to_s());

    printf("---------------------------------------------------------\n\n");
}

void CParamArray::clear(void) {
    _release(0);
}

CParamArray &CParamArray::operator=(const CParamArray &o) {
    int i, l;
    CVariant *v;

    // check if self-assignment
    if (this == &o) return *this;

    // clear old contents
    clear();

    // get item number
    l = o.na.size();

    // copy each item
    for (i = 0; i < l; i++) {
        v = new CVariant;
        *v = *(o.va[i]);

        na.push_back(o.na[i]);
        va.push_back(v);
    }

    // parse some field
    parse();

    return *this;
}

int CParamArray::_init(void) {
    // clear name & value array
    na.clear();
    va.clear();

    // clear stack array
    sa.clear();

    return 0;
}

int CParamArray::_release(int s) {
    unsigned long i, l;

    // free all variant objects
    l = na.size();

    for (i = 0; i < l; i++) {
        delete va[i];
    }

    // clear name & value list
    na.clear();
    va.clear();

    // clear stack objects
    if (s == 1) {
        for (i = 0; i < sa.size(); i++) delete sa[i];
        sa.clear();
    }

    return 0;
}

/*****************************************************************************\
\*****************************************************************************/

int CArgs::_init(void) {
    na.clear();

    return 0;
}

int CArgs::_release(void) {
    na.clear();

    return 0;
}

int CArgs::set_args(int argc, char *argv[]) {
    int i;

    na.clear();

    for (i = 0; i < argc; i++) {
        na.push_back(argv[i]);
    }

    return 0;
}

int CArgs::save(string fname) {
    string fn;
    FILE *fp;
    unsigned long i;
    tm *now;
    time_t t;
    char str_time[200];


    fn = fname + "_args.txt";
    fp = fopen(fn.c_str(), "a+");
    ASSERT(fp);

    // get current time
    time(&t);
    now = localtime(&t);
    strftime(str_time, 200, "%Y-%m-%d %H:%M:%S", now);

    fprintf(fp, "--------------- %s ---------------\n", str_time);

    for (i = 0; i < na.size(); i++)
        fprintf(fp, "%s ", na[i].c_str());

    fprintf(fp, "\n\n");

    fclose(fp);

    return 0;
}

