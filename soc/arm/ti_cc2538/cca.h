// (Created by [TheM])
#ifndef __CCA_H
#define __CCA_H


#define INTO_SECTION(x) __attribute__((section(x))) 
#define CCA_SECTION ".cca"

// reserved space pattern
#define CCA_OPT_RESERVED (0xFF)

// flash image validity marked
#define CCA_OPT_IMAGE_VALID (0x00)
#define CCA_OPT_IMAGE_INVALID (~CCA_CFG_IMAGE_VALID)

#define CCA_OPT_BACKDOOR_ENABLED (0x01)
#define CCA_OPT_BACKDOOR_DISABLED (0x00)

#define CCA_OPT_BACKDOOR_ACTIVEHIGH (0x01)
#define CCA_OPT_BACKDOOR_ACTIVELOW (0x00)

#define CCA_OPT_BACKDOOR_PIN0 (0x00)
#define CCA_OPT_BACKDOOR_PIN1 (0x01)
#define CCA_OPT_BACKDOOR_PIN2 (0x02)
#define CCA_OPT_BACKDOOR_PIN3 (0x03)
#define CCA_OPT_BACKDOOR_PIN4 (0x04)
#define CCA_OPT_BACKDOOR_PIN5 (0x05)
#define CCA_OPT_BACKDOOR_PIN6 (0x06)
#define CCA_OPT_BACKDOOR_PIN7 (0x07)

#define CCA_OPT_DEBUG_ACCESS_BLOCKED (0x00)
#define CCA_OPT_DEBUG_ACCESS_ALLOWED (0x01)

#define CCA_OPT_LOCKBITS_LOCKED (0x00)
#define CCA_OPT_LOCKBITS_UNLOCKED (0xFF)

#endif /* __CCA_H */
