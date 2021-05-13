/**
 * @file    xattr_table.cpp
 * @brief
 * @author  Egor Uleyskiy (e.uleyskiy@samsung.com)
 * @version 0.1
 * @date    Created 6 14, 2016
 * @par     In Samsung Ukraine R&D Center (SRK) under a contract between
 * @par     LLC "Samsung Electronics Ukraine Company" (Kiev, Ukraine) and
 * @par     "Samsung Electronics Co", Ltd (Seoul, Republic of Korea)
 * @par     Copyright: (c) Samsung Electronics Co, Ltd 2016. All rights reserved.
**/

#include <map>
#include <fstream>
#include <string>
#include <vector>
#include <new>
#include <list>
#include <stdint.h>
#include <cstring>

#include "xattr.h"
#include "xattr_table.h"
#include "base64_decode.h"

#define XATTR_SECURITY_PREFIX	"security."
#define XATTR_SECURITY_PREFIX_LEN (sizeof(XATTR_SECURITY_PREFIX) - 1)

#define XATTR_SYSTEM_PREFIX "system."
#define XATTR_SYSTEM_PREFIX_LEN (sizeof(XATTR_SYSTEM_PREFIX) - 1)

#define XATTR_TRUSTED_PREFIX "trusted."
#define XATTR_TRUSTED_PREFIX_LEN (sizeof(XATTR_TRUSTED_PREFIX) - 1)

#define XATTR_USER_PREFIX "user."
#define XATTR_USER_PREFIX_LEN (sizeof(XATTR_USER_PREFIX) - 1)

#define EXT4_XATTR_INDEX_USER     1
#define EXT4_XATTR_INDEX_TRUSTED  4
#define EXT4_XATTR_INDEX_SECURITY 6
#define EXT4_XATTR_INDEX_SYSTEM   7

using std::map;
using std::list;
using std::string;
using std::vector;
using std::ifstream;
using std::ios;
using std::nothrow;

class XattrTable
{
private:
    struct XattrEntry
    {
        unsigned index;
        string name;
        string fullname;
        vector<unsigned char> data;
    };

    typedef map<string, list<XattrEntry> > Table;

    Table table;

    bool add_to_list(list<XattrEntry>& l, const string& name, const vector<unsigned char>& data)
    {
        XattrEntry entry;

        if (name.compare(0, XATTR_SECURITY_PREFIX_LEN, XATTR_SECURITY_PREFIX) == 0)
        {
            entry.index = EXT4_XATTR_INDEX_SECURITY;
            entry.name = name.substr(XATTR_SECURITY_PREFIX_LEN);
            entry.fullname = name;
        }
        else if (name.compare(0, XATTR_SYSTEM_PREFIX_LEN, XATTR_SYSTEM_PREFIX) == 0)
        {
            entry.index = EXT4_XATTR_INDEX_SYSTEM;
            entry.name = name.substr(XATTR_SYSTEM_PREFIX_LEN);
            entry.fullname = name;
        }
        else if (name.compare(0, XATTR_TRUSTED_PREFIX_LEN, XATTR_TRUSTED_PREFIX) == 0)
        {
            entry.index = EXT4_XATTR_INDEX_TRUSTED;
            entry.name = name.substr(XATTR_TRUSTED_PREFIX_LEN);
            entry.fullname = name;
        }
        else if (name.compare(0, XATTR_USER_PREFIX_LEN, XATTR_USER_PREFIX) == 0)
        {
            entry.index = EXT4_XATTR_INDEX_USER;
            entry.name = name.substr(XATTR_USER_PREFIX_LEN);
            entry.fullname = name;
        }
        else
        {
            fprintf(stderr, "We don't support this xattr's namespace: %s\n", name.c_str());
            return false;
        }

        entry.data = data;
        l.push_back(entry);

        return true;
    }

    static bool comp(const XattrEntry& a, const XattrEntry &b)
    {
       if (a.index != b.index)
           return a.index < b.index;
       else if (a.name.size() != b.name.size())
           return a.name.size() < b.name.size();
       else
       {
           int ret = memcmp(a.name.c_str(), b.name.c_str(), a.name.size());
           if (ret != 0)
               return ret < 0;
       }

       // TODO: Check duplicated xattr while inserting in list
       fprintf(stderr, "Warning: found duplicated xattr: %s\n", a.name.c_str());
       return false;
    }

public:
    bool insert(const string& filepath, const string& name, const vector<unsigned char>& data)
    {
        return add_to_list(table[filepath], name, data);
    }

    bool open_file(const string& filepath)
    {
        ifstream file;
        file.open(filepath.c_str(), ios::in);
        if (!file.is_open())
            return false;

        string path;
        string attr_name;
        string attr_value;

        while (true)
        {
            file >> path;
            file >> attr_name;
            file >> attr_value;
            if (file.eof())
                break;

            vector<unsigned char> raw_attr(attr_value.size());
            size_t len = raw_attr.size();
            if (base64decode(attr_value.c_str(), attr_value.size(), &raw_attr.front(), &len) != 0 || len == 0)
            {
                fprintf(stderr, "Can't decode base64 value: path=%s attr_name=%s attr_value=%s\n",
                        path.c_str(), attr_name.c_str(), attr_value.c_str());
            }
            else
            {
                raw_attr.resize(len);
                insert(path, attr_name, raw_attr);
            }
        }
        return true;
    }

    bool setup_attrs(const string& filepath, nid_t inode, struct f2fs_sb_info *sbi)
    {
        Table::iterator attrs;

        attrs = table.find(filepath);
        if (attrs == table.end()) {
            return true;
        }

        attrs->second.sort(comp);

        for (list<XattrEntry>::iterator it = attrs->second.begin(); it != attrs->second.end(); ++it)
        {
            int ret = 0;

            ret = f2fs_setxattr(sbi, inode, it->index, it->name.c_str(), &it->data.front(), it->data.size(), 1);
            if (ret != 0)
                return false;
        }

        return true;
    }
};

void *xattr_table_open(const char *filepath)
{
    XattrTable *table = new(nothrow) XattrTable();

    if (!table)
        return NULL;

    if (table->open_file(filepath))
        return table;

    delete table;

    return NULL;
}

int xattr_table_add(void *table, const char *filepath, const char *name, const void *data, size_t len)
{
    if (!table)
        return -1;

    XattrTable *t = reinterpret_cast<XattrTable *>(table);
    vector<unsigned char> v_data(len);
    v_data.insert(v_data.begin(),
                  static_cast<const unsigned char *>(data), static_cast<const unsigned char *>(data) + len);

    if (!t->insert(filepath, name, v_data))
        return -2;

    return 0;
}

int xattr_table_setup(void *table, const char *filepath, nid_t inode, struct f2fs_sb_info *sbi)
{
    if (!table)
        return 0;

    XattrTable *t = reinterpret_cast<XattrTable *>(table);
    if (!t->setup_attrs(filepath, inode, sbi))
        return -1;

    return 0;
}

void xattr_table_close(void *table)
{
    if (!table)
        return;

    XattrTable *t = reinterpret_cast<XattrTable *>(table);
    delete t;
}
