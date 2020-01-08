/** \file
 * \brief Example code for reset Simple Open EtherCAT master
 *
 * Usage : reset [ifname1]
 * ifname is NIC interface, f.e. eth0
 *
 */

#include <denso_control/denso_client.h>
#include <ethercat_manager/ethercat_manager.h>
#include <stdio.h>

int main(int argc, char *argv[])
{
  printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

  if (argc > 1) {
    /* start slaveinfo */
    std::string                ifname(argv[1]);
    ethercat::EtherCatManager  manager(ifname);
    denso_control::DensoClient client(manager, 1, 0);

    // clear error
    client.reset();

  } else {
    printf("Usage: reset ifname1\nifname = eth0 for example\n");
  }

  printf("End program\n");

  return 0;
}
