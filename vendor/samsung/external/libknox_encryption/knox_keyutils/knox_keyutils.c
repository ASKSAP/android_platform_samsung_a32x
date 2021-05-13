/* keyutils.c: key utility library
 *
 * Copyright (C) 2005,2011 Red Hat, Inc. All Rights Reserved.
 * Written by David Howells (dhowells@redhat.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 * Copied & Modified @ 2017.12.12
 */

#include <stdarg.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <linux/keyctl.h>

typedef int32_t key_serial_t;

static long knox_keyctl(int cmd, ...)
{
    va_list va;
    unsigned long arg2, arg3, arg4, arg5;

    va_start(va, cmd);
    arg2 = va_arg(va, unsigned long);
    arg3 = va_arg(va, unsigned long);
    arg4 = va_arg(va, unsigned long);
    arg5 = va_arg(va, unsigned long);
    va_end(va);
    return syscall(__NR_keyctl, cmd, arg2, arg3, arg4, arg5);
}

key_serial_t knox_key_add(const char *type,
                     const char *description,
                     const void *payload,
                     size_t plen,
                     key_serial_t ringid)
{
    return syscall(__NR_add_key, type, description, payload, plen, ringid);
}

long knox_key_search(key_serial_t ringid, const char *type,
                   const char *description, key_serial_t destringid)
{
    return knox_keyctl(KEYCTL_SEARCH, ringid, type, description, destringid);
}
