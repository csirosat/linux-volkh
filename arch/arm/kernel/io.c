#include <asm/unaligned.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/io.h>

/*
 * Copy data from IO memory space to "real" memory space.
 * Optimized to use the most efficient access width: byte, word, or long.
 */
void _memcpy_fromio(void *to, const void __iomem *from, size_t count)
{
	const unsigned long  __iomem *fl;
	const unsigned short __iomem *fw;
	const unsigned char  __iomem *fb = from;
	unsigned long  *tl;
	unsigned short *tw;
	unsigned char  *tb = to;

	/* if count >= 4, and from & to both have same 4-byte alignment... */
	if ((count >= 4) && (((unsigned long)fb & 0x3) == ((unsigned long)tb & 0x3))) {
		/* first take care of any mis-alignment */
		while ((unsigned long)fb & 0x3) {
			count--;
			*tb++ = readb(fb++);
		}
		/* next copy by long-words */
		fl = (unsigned long *)fb;
		tl = (unsigned long *)tb;
		while (count >= 4) {
			count -= 4;
			*tl++ = readl(fl++);
		}
		/* copy pointers for tidy up below */
		fb = (unsigned char *)fl;
		tb = (unsigned char *)tl;
	} else
	/* if count >= 2, and from & to both have same 2-byte alignment... */
	if ((count >= 2) && (((unsigned long)fb & 0x1) == ((unsigned long)tb & 0x1))) {
		/* first take care of any mis-alignment */
		while ((unsigned long)fb & 0x1) {
			count--;
			*tb++ = readb(fb++);
		}
		/* next copy by short-words */
		fw = (unsigned short *)fb;
		tw = (unsigned short *)tb;
		while (count >= 2) {
			count -= 2;
			*tw++ = readw(fw++);
		}
		/* copy pointers for tidy up below */
		fb = (unsigned char *)fw;
		tb = (unsigned char *)tw;
	}
	/* tidy up any remaining bytes */
	while (count) {
		count--;
		*tb++ = readb(fb++);
	}
}

void _memcpy_fromiow(void *to, const void __iomem *from, size_t count)
{
	unsigned short *t = to;
	const unsigned short __iomem *f = from;

	BUG_ON(count % 2);
	while (count) {
		count -= 2;
		put_unaligned(get_unaligned(f), t);
		t++;
		f++;
	}
}

void _memcpy_fromiol(void *to, const void __iomem *from, size_t count)
{
	unsigned long *t = to;
	const unsigned long __iomem *f = from;

	BUG_ON(count % 4);
	while (count) {
		count -= 4;
		put_unaligned(get_unaligned(f), t);
		t++;
		f++;
	}
}

/*
 * Copy data from "real" memory space to IO memory space.
 * Optimized to use the most efficient access width: byte, word, or long.
 */
void _memcpy_toio(void __iomem *to, const void *from, size_t count)
{
	const unsigned long  *fl;
	const unsigned short *fw;
	const unsigned char  *fb = from;
	unsigned long  __iomem *tl;
	unsigned short __iomem *tw;
	unsigned char  __iomem *tb = to;

	/* if count >= 4, and from & to both have same 4-byte alignment... */
	if ((count >= 4) && (((unsigned long)fb & 0x3) == ((unsigned long)tb & 0x3))) {
		/* first take care of any mis-alignment */
		while ((unsigned long)fb & 0x3) {
			count--;
			writeb(*fb++, tb++);
		}
		/* next copy by long-words */
		fl = (unsigned long *)fb;
		tl = (unsigned long *)tb;
		while (count >= 4) {
			count -= 4;
			writel(*fl++, tl++);
		}
		/* copy pointers for tidy up below */
		fb = (unsigned char *)fl;
		tb = (unsigned char *)tl;
	} else
	/* if count >= 2, and from & to both have same 2-byte alignment... */
	if ((count >= 2) && (((unsigned long)fb & 0x1) == ((unsigned long)tb & 0x1))) {
		/* first take care of any mis-alignment */
		while ((unsigned long)fb & 0x1) {
			count--;
			writeb(*fb++, tb++);
		}
		/* next copy by short-words */
		fw = (unsigned short *)fb;
		tw = (unsigned short *)tb;
		while (count >= 2) {
			count -= 2;
			writeb(*fw++, tw++);
		}
		/* copy pointers for tidy up below */
		fb = (unsigned char *)fw;
		tb = (unsigned char *)tw;
	}
	/* tidy up any remaining bytes */
	while (count) {
		count--;
		writeb(*fb++, tb++);
	}
}

void _memcpy_toiow(void __iomem *to, const void *from, size_t count)
{
	const unsigned short *f = from;

	BUG_ON(count % 2);
	while (count) {
		count -= 2;
		writew(get_unaligned(f), to);
		f++;
		to += 2;
	}
}

void _memcpy_toiol(void __iomem *to, const void *from, size_t count)
{
	const unsigned long *f = from;

	BUG_ON(count % 4);
	while (count) {
		count -= 4;
		writel(get_unaligned(f), to);
		f++;
		to += 4;
	}
}

/*
 * "memset" on IO memory space.
 * This needs to be optimized.
 */
void _memset_io(void __iomem *dst, int c, size_t count)
{
	while (count) {
		count--;
		writeb(c, dst);
		dst++;
	}
}

EXPORT_SYMBOL(_memcpy_fromio);
EXPORT_SYMBOL(_memcpy_fromiow);
EXPORT_SYMBOL(_memcpy_fromiol);
EXPORT_SYMBOL(_memcpy_toio);
EXPORT_SYMBOL(_memcpy_toiow);
EXPORT_SYMBOL(_memcpy_toiol);
EXPORT_SYMBOL(_memset_io);
