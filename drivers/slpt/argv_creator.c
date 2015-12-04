/*
 *  Copyright (C) 2015 Wu Jiao <jiao.wu@ingenic.com wujiaososo@qq.com>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/slab.h>
#include "argv_creator.h"

#undef assert
#define assert(cond)											\
	do {														\
		if (!(cond)) {											\
			pr_err("#####assert condition failed (%s)#####\n", #cond);	\
			BUG_ON(1);											\
		}														\
	} while (0)

#define is_space_char(c) ((c) == ' ' || (c) == '\t' || (c) == '\n' || (c) == '\r')

static inline void *skip_space_char(const char *p) {
	for (; ;) {
		if (p[0] == '\0' || !is_space_char(p[0])) break; ++p;
		if (p[0] == '\0' || !is_space_char(p[0])) break; ++p;
		if (p[0] == '\0' || !is_space_char(p[0])) break; ++p;
		if (p[0] == '\0' || !is_space_char(p[0])) break; ++p;
	}

	if (p[0] == '\0')
		return NULL;
	return (void *)p;
}

static inline void *skip_nonspace_char(const char *p) {
	for (; ;) {
		if (p[0] == '\0' || is_space_char(p[0])) break; ++p;
		if (p[0] == '\0' || is_space_char(p[0])) break; ++p;
		if (p[0] == '\0' || is_space_char(p[0])) break; ++p;
		if (p[0] == '\0' || is_space_char(p[0])) break; ++p;
	}

	return (void *)p;
}

struct argv_creator *alloc_argv_creator(char *buffer) {
	struct argv_creator *ac;
	char *p;
	char *ep;
	unsigned int argc = 0;

	if (!buffer) {
		pr_err("%s: invalid args\n", __func__);
		assert(0);
	}

	ac = kmalloc(sizeof(*ac), GFP_KERNEL);
	if (!ac)
		return NULL;

	ac->buffer = buffer;
	ac->save_ec = NULL;
	ac->argv = NULL;
	ac->argc = 0;

	/* count for argc */
	p = buffer;
	while (1) {
		p = skip_space_char(p);		/* find the start of a string */
		if (!p)
			break;
		ep = skip_nonspace_char(p); /* find the end of a string */
		p = ep;
		argc++;
	}

	if (argc == 0)
		return ac;

	/* malloc the char *argv[] */
	ac->argc = argc;
	ac->argv = kmalloc((sizeof(*ac->argv) + sizeof(*ac->save_ep) + sizeof(*ac->save_ec)) * argc, GFP_KERNEL);
	if (!ac->argv) {
		kfree(ac);
		return NULL;
	}
	ac->save_ep = (void *)ac->argv + sizeof(*ac->argv) * argc;
	ac->save_ec = (void *)ac->argv + (sizeof(*ac->argv) + sizeof(*ac->save_ep)) * argc;

	/* give the argv[i] value */
	argc = 0;
	p = buffer;
	while (1) {
		p = skip_space_char(p);		/* find the start of a string */
		if (!p)
			break;
		ep = skip_nonspace_char(p); /* find the end of a string */
		ac->argv[argc] = p;
		ac->save_ep[argc] = ep;
		ac->save_ec[argc] = *ep;
		p = ep + 1;
		argc++;
		if (*ep == '\0')
			break;
		*ep = '\0';
	}

	return ac;
}

void argv_creator_free(struct argv_creator *ac) {
	unsigned int i;

	if (ac) {
		if (ac->argv) {
			for (i = 0; i < ac->argc; ++i) {
				*ac->save_ep[i] = ac->save_ec[i];
			}
			kfree(ac->argv);
		}
		kfree(ac);
	}
}
