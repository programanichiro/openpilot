#include <cassert>

#include <kaitai/kaitaistream.h>

#include "cereal/messaging/messaging.h"
#include "common/swaglog.h"
#include "common/util.h"
#include "system/ubloxd/ublox_msg.h"

ExitHandler do_exit;
using namespace ublox;

int main() {
  LOGW("starting ubloxd");
  AlignedBuffer aligned_buf;
  UbloxMsgParser parser;

  PubMaster pm({"ubloxGnss", "gpsLocationExternal"});

  std::unique_ptr<Context> context(Context::create());
  std::unique_ptr<SubSocket> subscriber(SubSocket::create(context.get(), "ubloxRaw"));
  assert(subscriber != NULL);
  subscriber->setTimeout(100);

  int gps_check = 100;
  FILE *fp = fopen("/dev/shm/gps_check.txt","w");
  if(fp){
    fprintf(fp,"#%d",1);
    fclose(fp);
  }

  while (!do_exit) {
    gps_check++;

    fp = fopen("/dev/shm/gps_check.txt","w");
    if(fp){
      fprintf(fp,"#%d",gps_check + 100000);
      fclose(fp);
    }

    std::unique_ptr<Message> msg(subscriber->receive());
    if (!msg) {
      continue;
    }

    fp = fopen("/dev/shm/gps_check2.txt","w");
    if(fp){
      fprintf(fp,"#%d",gps_check + 200000);
      fclose(fp);
    }

    capnp::FlatArrayMessageReader cmsg(aligned_buf.align(msg.get()));
    cereal::Event::Reader event = cmsg.getRoot<cereal::Event>();
    auto ubloxRaw = event.getUbloxRaw();
    float log_time = 1e-9 * event.getLogMonoTime();

    const uint8_t *data = ubloxRaw.begin();
    size_t len = ubloxRaw.size();
    size_t bytes_consumed = 0;

    while (bytes_consumed < len && !do_exit) {

      fp = fopen("/dev/shm/gps_check3.txt","w");
      if(fp){
        fprintf(fp,"#%d",gps_check + 300000);
        fclose(fp);
      }

      size_t bytes_consumed_this_time = 0U;
      if (parser.add_data(log_time, data + bytes_consumed, (uint32_t)(len - bytes_consumed), bytes_consumed_this_time)) {

        fp = fopen("/dev/shm/gps_check4.txt","w");
        if(fp){
          fprintf(fp,"#%d",gps_check + 400000);
          fclose(fp);
        }

        try {
          auto ublox_msg = parser.gen_msg();

          fp = fopen("/dev/shm/gps_check5.txt","w");
          if(fp){
            fprintf(fp,"#%d",gps_check + 500000);
            fclose(fp);
          }

          if (ublox_msg.second.size() > 0) {
            auto bytes = ublox_msg.second.asBytes();
            pm.send(ublox_msg.first.c_str(), bytes.begin(), bytes.size());
          }
        } catch (const std::exception& e) {

          fp = fopen("/dev/shm/gps_check6.txt","w");
          if(fp){
            fprintf(fp,"#%d",gps_check + 600000);
            fclose(fp);
          }

          LOGE("Error parsing ublox message %s", e.what());
        }

        parser.reset();
      }
      bytes_consumed += bytes_consumed_this_time;
      fp = fopen("/dev/shm/gps_check7.txt","w");
      if(fp){
        fprintf(fp,"#%d",gps_check + 700000);
        fclose(fp);
      }
    }
    fp = fopen("/dev/shm/gps_check8.txt","w");
    if(fp){
      fprintf(fp,"#%d",gps_check + 800000);
      fclose(fp);
    }

  }
  fp = fopen("/dev/shm/gps_check9.txt","w");
  if(fp){
    fprintf(fp,"#%d",gps_check + 900000);
    fclose(fp);
  }

  return 0;
}
