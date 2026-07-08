#ifndef INC_VERSION_H_
#define INC_VERSION_H_

#include <stdint.h>

/* --- Software Version (Semantic Versioning: MAJOR.MINOR.PATCH) --- */
#define SW_VERSION_MAJOR      0
#define SW_VERSION_MINOR      0
#define SW_VERSION_PATCH      1

/* --- Function Prototypes --- */
void Version_BlinkSequence(void);
uint32_t getUnique32bitID(void);
void getUnique96bitID(uint32_t *id_buffer);

#endif /* INC_VERSION_H_ */
