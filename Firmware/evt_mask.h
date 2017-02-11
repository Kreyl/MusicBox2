/*
 * evt_mask.h
 *
 *  Created on: Apr 12, 2013
 *      Author: g.kruglov
 */

#ifndef EVT_MASK_H_
#define EVT_MASK_H_

// Event masks
#define EVT_UART_NEW_CMD        EVENT_MASK(1)
#define EVT_USB_READY           EVENT_MASK(2)
#define EVT_USB_CONNECTED       EVENT_MASK(3)
#define EVT_USB_DISCONNECTED    EVENT_MASK(4)
#define EVT_PLAY_ENDS           EVENT_MASK(5)
#define EVT_BUTTONS             EVENT_MASK(6)
#define EVT_ADC_DONE            EVENT_MASK(7)

#define EVT_BOX1_CLOSED         EVENT_MASK(10)
#define EVT_BOX2_CLOSED         EVENT_MASK(11)
#define EVT_TMR_BigVolUp        EVENT_MASK(12)
#define EVT_TMR_BigVolDown      EVENT_MASK(13)


#endif /* EVT_MASK_H_ */
