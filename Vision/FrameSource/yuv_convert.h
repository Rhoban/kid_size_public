#ifndef _YUV_CONVERT_H
#define _YUV_CONVERT_H

extern "C" {
/**
 * YUV conversion (deinterlacing)
 */
void yuv_convert(char* input, char* output, int pixels);
}

#endif
