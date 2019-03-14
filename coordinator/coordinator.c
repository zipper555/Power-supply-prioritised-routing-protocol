/*
*******************  Coordinator Node ******************
*/

// Contiki-specific includes:
#include "contiki.h"
#include "net/rime/rime.h"	// Establish connections.
#include "net/netstack.h"
#include "random.h"
#include "dev/leds.h"
#include "core/net/linkaddr.h"
#include "dev/adc-zoul.h"      // ADC
#include "dev/zoul-sensors.h"  // Sensor functions

// Standard C includes:
#include <stdio.h>
#include <stdint.h>


#define BROADCAST_RIME_CHANNEL 129
#define BROADCAST_INTERVAL 10
#define SENSEVAL_INTERVAL 15 
#define CHANNEL 13
#define POWER 3
#define MY_ADDR 2
#define MAX_PAYLOAD 80
#define MAX_N 20
//#define NETSTACK_CONF_MAC csma_driver

typedef struct{
	uint8_t msg_type;
	uint8_t node_id;
	uint8_t priority;
	uint8_t src;
	uint8_t sensor_val;
	uint16_t battery_val;
	uint8_t spl_brdcst_reply;
	linkaddr_t dst;
	linkaddr_t 	next_hop;
}packet;

//Global variables
static uint8_t self_node_id;
static packet tx_packet, rx_packet, tx_packet_uc, rx_packet_uc;

/* Functions */
static void print_received_packet();
static void set_init_brdcst_param();

// Definition of Processes
PROCESS(homeauto_coord_process, "Coordinator Node");
PROCESS(homeauto_coord_uc_process, "Coordinator Node Unicast");

/*
 +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   	   	   	   	   	   	   	  BROADCAST CONNECTION
 +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */

// Creates an instance of a broadcast connection.
static struct broadcast_conn broadcast;
static struct unicast_conn unicast;

// Defines the behavior of a connection upon receiving data.
static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
	int16_t rssi;
	leds_on(LEDS_GREEN);

	printf("\nBroadcast Routing message received from 0x%x%x: \n\r [RSSI %d]\n\r",
			from->u8[0], from->u8[1],
			packetbuf_attr(PACKETBUF_ATTR_RSSI));
	rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);
	printf("RSSI = %d\n", rssi);

	packetbuf_copyto(&rx_packet);
	print_received_packet(rx_packet);

	leds_off(LEDS_GREEN);
}

static void unicast_recv(struct unicast_conn *c, const linkaddr_t *from)
{
  int16_t rssi;
  printf("\nUnicast message received from 0x%x%x: \n\r [RSSI %d]\n\r",
  			from->u8[0], from->u8[1], packetbuf_attr(PACKETBUF_ATTR_RSSI));
  rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);
  printf("RSSI = %d\n", rssi);

  /* For the demo */
  	if (rssi < -85) {
  		return;
  	}
  	/******************/

  packetbuf_copyto(&rx_packet_uc);
  print_received_packet(rx_packet_uc);
  /* To do: Blink LED depending on source - rx_packet.src */
}

// Defines the functions used as callbacks for a broadcast connection.
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static const struct unicast_callbacks unicast_callbacks = {unicast_recv};

/*
 +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                                    MAIN PROCESS
 +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */

AUTOSTART_PROCESSES(&homeauto_coord_process, &homeauto_coord_uc_process);

PROCESS_THREAD(homeauto_coord_process, ev, data)
{
	static uint8_t broadcast_time_interval = BROADCAST_INTERVAL;
	static struct etimer broadcast_table_timer;
	self_node_id = (linkaddr_node_addr.u8[0] & 0xFF);
	printf("Self Node ID is: 0x%x\n", self_node_id);

	PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
    	PROCESS_BEGIN();
	
	/*
	 * set your group's channel
	 */
	NETSTACK_CONF_RADIO.set_value(RADIO_PARAM_CHANNEL, 13);

	/*
	 * Change the transmission power
	 */
	NETSTACK_CONF_RADIO.set_value(RADIO_PARAM_TXPOWER, -24);

	// Open broadcast connection.
	broadcast_open(&broadcast, BROADCAST_RIME_CHANNEL, &broadcast_call);

	//static int i;

	while(1) //MAIN LOOP
	{
		
		// Set values for initial broadcast
		set_init_brdcst_param();	
		
		leds_on(LEDS_RED);
		//Copy the content of tx_packet to the buffer.
		
		packetbuf_copyfrom(&tx_packet, sizeof(packet));
		//printf("\n Printing packetbuf soon after copying struct\n");
		//printf("%s\n", packetbuf_dataptr());

		broadcast_send(&broadcast);
		leds_off(LEDS_RED);
		printf("PACKET SENT FROM COORD Rime addr: 0x%x%x\n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);

		etimer_set(&broadcast_table_timer, CLOCK_SECOND * broadcast_time_interval);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&broadcast_table_timer));
	}
	PROCESS_END();
	return 0;
}

/* Process to handle Unicast connections */
PROCESS_THREAD(homeauto_coord_uc_process, ev, data)
{

	PROCESS_EXITHANDLER(unicast_close(&unicast));
	PROCESS_BEGIN();
	unicast_open(&unicast, 146, &unicast_callbacks);
	
	static struct etimer senseval_timer;
	etimer_set(&senseval_timer, SENSEVAL_INTERVAL * CLOCK_SECOND);

	while (1) {
		PROCESS_YIELD();
		/* Nothing to be done here for the Coordinator Node 
		* 
		*/
		etimer_set(&senseval_timer, SENSEVAL_INTERVAL * CLOCK_SECOND);
	}
	PROCESS_END();
	return 0;
}

static void set_init_brdcst_param()
{
	//self_node_id = (linkaddr_node_addr.u8[0] & 0xFF);
	printf("Generating INIT BROADCAST packet\n");
	tx_packet.msg_type = 0x01;
	tx_packet.node_id = self_node_id;
	/* Coordinator node priority = 0x00 */
	tx_packet.priority = 0x00;
	tx_packet.src = self_node_id;
	tx_packet.sensor_val = 0x55;
	tx_packet.battery_val = 0xff;
	tx_packet.spl_brdcst_reply = 0x00;
	tx_packet.dst.u8[0] = 0x00;
}

static void print_received_packet(packet recv_pack)
{
	//int len = packetbuf_copyto(&rx_packet);
	printf("\nPrinting received packet\n");
	printf("Msg type: 0x%x\n", recv_pack.msg_type);
	printf("Node ID: 0x%x\n", recv_pack.node_id);
	printf("Priority: 0x%x\n", recv_pack.priority);
	printf("Source: 0x%x\n", recv_pack.src);
	printf("Sensor Value: 0x%x\n", recv_pack.sensor_val);
	printf("Battery Value: 0x%x\n", recv_pack.battery_val);
	printf("Special broadcast reply: 0x%x\n", recv_pack.spl_brdcst_reply);
	printf("Destination: 0x%x\n", recv_pack.dst.u8[0]);
	printf("Next hop: 0x%x\n", recv_pack.next_hop.u8[0]);
}
