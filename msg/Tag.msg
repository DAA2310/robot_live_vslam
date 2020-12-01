# tag ID and size
int32 id
float64 size

# flag indicators for processed and optimized tag
bool avg
bool opt

# world T tag of tag, 0-2 is rodrigues rotation vector and 3-5 is translation
float32[6] wTtag
