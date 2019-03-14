/*
****************** Battery Node ******************
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
#define INITBRDCST_INTERVAL 10
#define ROUTING_TABLE_VALID_INTERVAL 15
#define SENSEVAL_INTERVAL 5
#define NODE_UNREACHABLE_THRESHOLD 5
#define CHANNEL 14
#define POWER 3
#define MY_ADDR 2
#define MAX_PAYLOAD 80
#define MAX_N 20
//#define NETSTACK_CONF_MAC csma_driver

/* Message Types */
#define MSG_INIT_BROADCAST	0x01
#define MSG_REBROADCAST		0x02
#define MSG_SENSOR_VAL		0x03
#define MSG_SPL_BROADCAST	0x04

/*TX PACKET GENERATION */
#define TX_PACKET_GEN_SENSOR_VAL	0x01
#define TX_PACKET_GEN_FORWARD		0x02
#define SPL_BRDCST_REQUEST		0x03
#define SPL_BRDCST_REPLY		0x04



/* Packet Definition */
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

/* Routing Table */
typedef struct
{
	linkaddr_t 	dest; // This is always the coordinator in our case
	linkaddr_t 	next_hop;	// Best choice for next hop
	uint8_t 	priority; 
	uint8_t 	expired; // Set a timer to turn this flag to TRUE after a stipulated time. Addresses Neighboring node failures.
}r_table;

//Global variables
static uint8_t self_node_id;
static uint8_t local_sensor_value;
static packet tx_packet, rx_packet, tx_packet_uc, rx_packet_uc, rx_packet_uc_splbrdcst;
process_event_t SPL_BRDCST, SPL_BRDCST_UC;

static r_table table = {
	.dest.u8[0] = 0x1,
	.next_hop.u8[0] = 0xff, .next_hop.u8[1] = 0x00,
	.priority = 0xff,
	.expired = 0x1,
};
//static uint8_t REBRDCST = 0x00;
static uint8_t INITBRDCST_COMPLETE = 0x00;
static int r_table_expiry_count = 0; // Used to check if a node is detached


/* Timers */
static struct ctimer initbrdcst_complete_ctimer;
static struct ctimer routing_table_expiry_ctimer;

/* Functions */
static void generate_tx_packet_uc (int tx_packet_type);
static void send_uc_packet();
static void print_received_packet();
static void set_init_brdcst_param();
static void routing_table_update();
static void routing_table_print();
static uint16_t get_battery_val();
static void generate_spl_brdcst_tx_packet();
static int getDistanceSensorValue(float m, float b, uint16_t adc_input);
static int getHumiditySensorValue(float m, float b, uint16_t adc_input);



static void initbrdcst_complete_callback(void *ptr)
{
	INITBRDCST_COMPLETE = 0x00;
	ctimer_reset(&initbrdcst_complete_ctimer);
	printf("INIT BROADCAST expired\n");
}

static void routing_table_expiry_callback(void *ptr)
{
	table.expired = 0x1;
	ctimer_reset(&routing_table_expiry_ctimer);
	printf("Routing Table expired\n");
	r_table_expiry_count ++;
}

// Definition of Processes
PROCESS(homeauto_batterynode_process, "Battery Node");
PROCESS(homeauto_batterynode_uc_process, "Battery Node Unicast");

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

	printf("Broadcast Routing message received from 0x%x%x: \n\r [RSSI %d]\n\r",
			from->u8[0], from->u8[1],
			packetbuf_attr(PACKETBUF_ATTR_RSSI));
	
	rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);
	printf("RSSI = %d\n", rssi);


	if (rssi < -70) {
		leds_off(LEDS_GREEN);
		/* For the demo */
		return;

	} else {
	/******************/

	packetbuf_copyto(&rx_packet);
	print_received_packet(rx_packet);
	leds_off(LEDS_GREEN);
	/* Update the routing table if Initbrdcst time has expired or Brdcst from a higher priority node is heard */
	//if (rx_packet.msg_type == 0x01 && INITBRDCST_COMPLETE == 0x00) {
	if (rx_packet.msg_type == 0x01 && (rx_packet.priority < table.priority || INITBRDCST_COMPLETE == 0x00)) {
		printf("RECEIVED INIT BRDCST\n");
		// Update Routing table only if a higher priority node is reachable
		// or if the Routing entry has expired.
		if (rx_packet.priority <= table.priority || table.expired == 0x1 ) {
			routing_table_update(rx_packet);
		}	
		routing_table_print();	
		INITBRDCST_COMPLETE = 0x01; 
		//setup a callback timer to set this to 0x00
		//No rebroadcasting for Battery nodes.
	} else if (rx_packet.msg_type == MSG_SPL_BROADCAST) {
		printf("RECEIVED SPL BROADCAST REQUEST\n");
		/* Reply only if the routing table of self is valid */
		if (r_table_expiry_count < NODE_UNREACHABLE_THRESHOLD) {
			rx_packet_uc_splbrdcst = rx_packet; //We use this packet from now
			process_post(&homeauto_batterynode_uc_process, SPL_BRDCST_UC, 0);
		}

	}
	}
}

static void unicast_recv(struct unicast_conn *c, const linkaddr_t *from)
{
  int16_t rssi;
  printf("Unicast msg received from %d.%d\n",
           from->u8[0], from->u8[1]);
  rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);
  printf("RSSI = %d\n", rssi);

  packetbuf_copyto(&rx_packet_uc);
  print_received_packet(rx_packet_uc);

  if(rx_packet_uc.msg_type == MSG_SENSOR_VAL) {
    printf("unicast msg received from %d.%d\n",
           from->u8[0], from->u8[1]);
    /* Forward the Packet to next hop */
    generate_tx_packet_uc(TX_PACKET_GEN_FORWARD);
    send_uc_packet(table.next_hop);
  } else if (rx_packet_uc.msg_type == MSG_SPL_BROADCAST &&
			rx_packet_uc.spl_brdcst_reply == 0x01) {
  printf("Special broadcast reply received\n");
  routing_table_update(rx_packet_uc);
  routing_table_print();
  }
}

// Defines the functions used as callbacks for a broadcast and unicast connections.
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static const struct unicast_callbacks unicast_callbacks = {unicast_recv};

/*
 +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                                    MAIN PROCESS
 +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */

AUTOSTART_PROCESSES(&homeauto_batterynode_process, &homeauto_batterynode_uc_process);

PROCESS_THREAD(homeauto_batterynode_process, ev, data)
{
	static uint8_t broadcast_time_interval = BROADCAST_INTERVAL;
	static uint8_t initbrdcst_interval = INITBRDCST_INTERVAL;
	static uint8_t routing_table_valid_interval = ROUTING_TABLE_VALID_INTERVAL;
	static struct etimer broadcast_table_timer;

	PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
    PROCESS_BEGIN();
	
	self_node_id = (linkaddr_node_addr.u8[0] & 0xFF);
	printf("Linkaddr of Battery Node: u8[1]= 0x%x u8[0]= 0x%x\n", linkaddr_node_addr.u8[1], linkaddr_node_addr.u8[0]);
	printf("Self Node ID is: 0x%x\n", self_node_id);
	ctimer_set(&initbrdcst_complete_ctimer, CLOCK_SECOND * initbrdcst_interval, 				initbrdcst_complete_callback, NULL);
	ctimer_set(&routing_table_expiry_ctimer, 
		CLOCK_SECOND * routing_table_valid_interval, routing_table_expiry_callback, 			NULL);
	
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
		/* No rebroadcasting for Battery nodes */
		if (ev == SPL_BRDCST) {
				//printf("SPL BROADCAST event received\n");
				generate_spl_brdcst_tx_packet(SPL_BRDCST_REQUEST);
				packetbuf_copyfrom(&tx_packet, sizeof(packet));
				broadcast_send(&broadcast);
				printf("SPL_BRDCST packet sent from battery node: 0x%x\n", self_node_id);
		}

		
		/* Sense value and send 
		 * If the routing table is empty, then initiate a special broadcast.
		 * TO DO...
		 */
		
		etimer_set(&broadcast_table_timer, CLOCK_SECOND * broadcast_time_interval);
		//Sense val interval should be lesser than the interval of init broadcast
		//etimer_set(&senseval_timer, CLOCK_SECOND * senseval_interval);
		//PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&senseval_timer));
		PROCESS_YIELD();
	}
	PROCESS_END();
	return 0;
}

PROCESS_THREAD(homeauto_batterynode_uc_process, ev, data)
{
	static uint16_t adc1_value;
	int sen_val,hum_val,dis_val;
	uint8_t tem_val;

	PROCESS_EXITHANDLER(unicast_close(&unicast));
	PROCESS_BEGIN();
	unicast_open(&unicast, 146, &unicast_callbacks);
	
	static struct etimer senseval_timer;
	etimer_set(&senseval_timer, SENSEVAL_INTERVAL * CLOCK_SECOND);

	adc_zoul.configure(SENSORS_HW_INIT, ZOUL_SENSORS_ADC1);

	while (1) {
		PROCESS_YIELD();
		/* READ THE SENSOR VALUE 
		* To be adapted based on the sensor.
		*For now assigning a random hardcoded value.
		*/
		if (ev == SPL_BRDCST_UC) {
					printf("Special broadcast request at UC Process\n");
					generate_tx_packet_uc(SPL_BRDCST_REPLY);
					//print_received_packet(tx_packet_uc);
					send_uc_packet(tx_packet_uc.dst);
		} else {

		if (self_node_id == 0x05) {
			//Temperature
			printf("Reading Temperature...\n");
			adc1_value = adc_zoul.value(ZOUL_SENSORS_ADC1) >> 4;
		    sen_val = cc2538_temp_sensor.value(CC2538_SENSORS_VALUE_TYPE_CONVERTED);
			tem_val = (unsigned int)sen_val/1000;
			local_sensor_value = tem_val;
		} else if (self_node_id == 0x06) {
			//Humidity
			printf("Reading Humidity...\n");
			adc1_value = adc_zoul.value(ZOUL_SENSORS_ADC1) >> 4;
			hum_val = getHumiditySensorValue(190.6, 40.2, adc1_value);
			printf("Humidity:%d\n",hum_val);
			local_sensor_value = hum_val;
		} else if (self_node_id == 0x07) {
			//Distance
			printf("Reading Distance...\n");
			adc1_value = adc_zoul.value(ZOUL_SENSORS_ADC1) >> 4;
			dis_val = getDistanceSensorValue(4.8,0.02,adc1_value);
			local_sensor_value = dis_val;
		} else {
			printf("Invalid Node ID\n");
		}
			
		//local_sensor_value = 0xab;
		//printf("Sensor value = 0x%x\n", local_sensor_value);
		generate_tx_packet_uc(TX_PACKET_GEN_SENSOR_VAL);
		send_uc_packet(table.next_hop);
		
		etimer_set(&senseval_timer, SENSEVAL_INTERVAL * CLOCK_SECOND);
		}
	}
	PROCESS_END();
	return 0;
}

/* Generates Unicast tx packet depending on the msg_type */
static void generate_tx_packet_uc (int tx_packet_type) 
{	
	switch (tx_packet_type) {
	case TX_PACKET_GEN_SENSOR_VAL:
		printf("Generat ing TX PACKET for Sensor val\n");
		tx_packet_uc.msg_type = MSG_SENSOR_VAL;
		tx_packet_uc.node_id = self_node_id;
		/* Battery node priority = 0x03 */
		tx_packet_uc.priority = 0x03;
		tx_packet_uc.src = self_node_id; // Source is self for a generated packet.
		tx_packet_uc.sensor_val = local_sensor_value;
		tx_packet_uc.battery_val = get_battery_val();
		tx_packet_uc.spl_brdcst_reply = 0x00;
		tx_packet_uc.dst.u8[0] = 0x1; //Destination is coordinator
		tx_packet_uc.next_hop = table.next_hop;
		break;
	case TX_PACKET_GEN_FORWARD:
		printf("Generating TX PACKET for Unicast forwarding\n");
		/* Only node_id of the sender needs to be changed. Rest of the information forwarded */
		tx_packet_uc = rx_packet_uc;
		tx_packet_uc.node_id = self_node_id;
		break;
	case SPL_BRDCST_REPLY:
		printf("Generating TX PACKET for Spl broadcast reply\n");
		tx_packet_uc = rx_packet_uc_splbrdcst;
		tx_packet_uc.node_id = self_node_id;
		tx_packet_uc.spl_brdcst_reply = 0x01;
		/* Destination is the Requesting node */
		tx_packet_uc.dst.u8[0] = rx_packet_uc_splbrdcst.node_id;
		break;
	default: printf("Invalid msg_type\n");
	}
}

/* Send the constructed Unicast packet to the next hop */
static void send_uc_packet(linkaddr_t destination)
{
	if (r_table_expiry_count > NODE_UNREACHABLE_THRESHOLD) {
		printf("!!! Routing Table not updated, Node unreachable from broadcasts !!!\n");
		leds_toggle(LEDS_RED); //Alarm LED
		process_post(&homeauto_batterynode_process, SPL_BRDCST, 0);
	} else {
		printf("Sending Unicast packet\n");
		leds_on(LEDS_BLUE);
		packetbuf_copyfrom(&tx_packet_uc, sizeof(packet));
		//Send out the packet to next hop
		unicast_send(&unicast, &destination);
		leds_off(LEDS_BLUE);
	}
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
	printf("Next hop at Source: 0x%x\n", recv_pack.next_hop.u8[0]);
}

/* This function not used for Battery Nodes */
static void set_init_brdcst_param()
{
	self_node_id = (linkaddr_node_addr.u8[0] & 0xFF);
	tx_packet.msg_type = 0x01;
	tx_packet.node_id = self_node_id;
	tx_packet.priority = 0x02;
	tx_packet.src = self_node_id;
	tx_packet.sensor_val = 0x55;
	tx_packet.battery_val = 0xff;
	tx_packet.spl_brdcst_reply = 0x00;
	tx_packet.dst.u8[0] = 0x00;
}

static void generate_spl_brdcst_tx_packet(int spl_brdst_type)
{

	tx_packet.msg_type = MSG_SPL_BROADCAST;
	tx_packet.node_id = self_node_id;
	/* Priority of Battery Node = 0x03 */
	tx_packet.priority = 0x03;
	tx_packet.src = self_node_id;
	tx_packet.sensor_val = 0x00;
	tx_packet.battery_val = get_battery_val();
	tx_packet.spl_brdcst_reply = 0x00;
	tx_packet.dst.u8[0] = 0x00;

}

static void routing_table_update(packet recv_pack)
{
//Use rx_packet
	printf("Updating Routing Table\n");
	table.dest.u8[0] = 0x01;
	table.next_hop.u8[0] = recv_pack.node_id;
	table.priority = recv_pack.priority;
	table.expired = 0x00;
	/* As the timer hasn't expired yet, resetting can't be done. Restart it. */
	ctimer_restart(&routing_table_expiry_ctimer);	
	r_table_expiry_count = 0;
	leds_off(LEDS_RED); // Switch off the alarm LED

}

static void routing_table_print()
{
	printf("Routing table:\n");
	printf("----------------------------------\n");
	printf("ID| DESTINATION | NEXT_HOP | PRIORITY \n");
	printf("0x%x | 0x1     | 0x%x     | %d        \n", self_node_id, table.next_hop,
			table.priority);
	printf("-----------------------------------\n");
}

static uint16_t get_battery_val()
{
	uint16_t voltage;
	voltage = vdd3_sensor.value(CC2538_SENSORS_VALUE_TYPE_CONVERTED);
	return voltage;
}

static int getHumiditySensorValue(float m, float b, uint16_t adc_input) {
	float sensor_val, hum;
	sensor_val = (float)adc_input / 2047.0;
	hum = m * sensor_val - b;
	return (int)hum;
}

static int getDistanceSensorValue(float m, float b, uint16_t adc_input) {
	float sensor_val, dis;
	sensor_val = (float)adc_input / 2047.0;
	dis = m/(sensor_val-b);
	return (int)dis;
}
