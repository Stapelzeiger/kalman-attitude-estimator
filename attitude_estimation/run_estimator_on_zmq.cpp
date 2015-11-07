#include <iostream>
#include "template_kalman.h"
#include "ekf_gyro_acc.h"
#include <zmq.hpp>
#include "cmp/cmp.h"
#include "cmp_mem_access/cmp_mem_access.h"

int main(int argc, char const *argv[])
{
    zmq::context_t context(1);

    zmq::socket_t subscriber (context, ZMQ_SUB);
    subscriber.connect("tcp://localhost:5556");

    subscriber.setsockopt(ZMQ_SUBSCRIBE, NULL, 0);

    EKFGyroAcc kalman;
    sleep(3);
    float prev_timestamp = 0;
    int i = 1000;
    while (1) {
        zmq::message_t update;

        subscriber.recv(&update);
        if (i > 0) {
            i--;
            continue;
        }

        cmp_ctx_t cmp;
        cmp_mem_access_t mem;
        cmp_mem_access_ro_init(&cmp, &mem, update.data(), update.size());

        char strbuf[100];
        bool err = false;
        uint32_t strsize = sizeof(strbuf);
        err = err || !cmp_read_str(&cmp, &strbuf[0], &strsize);
        if (err) {
            continue;
        }
        // std::cout << "received: " << strbuf << std::endl;
        if (strcmp("imu", strbuf) == 0) {
            uint32_t mapsize = 0;
            err = err | !cmp_read_map(&cmp, &mapsize);
            bool gyro_ok = false;
            bool acc_ok = false;
            bool time_ok = false;
            float gyro[3];
            float acc[3];
            float timestamp;
            for (int i = 0; i < mapsize; i++) {
                uint32_t strsize = sizeof(strbuf);
                err = err || !cmp_read_str(&cmp, strbuf, &strsize);
                uint32_t arr_sz;
                if (!err && strcmp("gyro", strbuf) == 0) {
                    err = err || !cmp_read_array(&cmp, &arr_sz);
                    err = err || (arr_sz != 3);
                    err = err || !cmp_read_float(&cmp, &gyro[0]);
                    err = err || !cmp_read_float(&cmp, &gyro[1]);
                    err = err || !cmp_read_float(&cmp, &gyro[2]);
                    gyro_ok = !err;
                } else if (!err && strcmp("acc", strbuf) == 0) {
                    err = err || !cmp_read_array(&cmp, &arr_sz);
                    err = err || (arr_sz != 3);
                    err = err || !cmp_read_float(&cmp, &acc[0]);
                    err = err || !cmp_read_float(&cmp, &acc[1]);
                    err = err || !cmp_read_float(&cmp, &acc[2]);
                    acc_ok = !err;
                } else if (!err && strcmp("time", strbuf) == 0) {
                    err = err || !cmp_read_float(&cmp, &timestamp);
                    time_ok = !err;
                }
            }

            if (!err && gyro_ok && acc_ok && time_ok) {
                acc[0] = acc[0]/1;
                acc[1] = acc[1]/1;
                acc[2] = acc[2]/1;

                if (prev_timestamp == 0) {
                    prev_timestamp = timestamp;
                    continue;
                }
                float delta_t = timestamp - prev_timestamp;
                prev_timestamp = timestamp;

                kalman.update_imu(gyro, acc, delta_t);

                std::cout << kalman.get_attitude().w() << ", "
                        << kalman.get_attitude().x() << ", "
                        << kalman.get_attitude().y() << ", "
                        << kalman.get_attitude().z() << std::endl;
                // std::cerr << "q: " << kalman.get_attitude().w() << ", "
                //         << kalman.get_attitude().x() << ", "
                //         << kalman.get_attitude().y() << ", "
                //         << kalman.get_attitude().z() << std::endl;

                // std::cerr << kalman.P << std::endl;
                // std::cerr << "time: " << timestamp << std::endl;

                // std::cerr << kalman.P(0, 0) << ", "
                //         << kalman.P(1, 1) << ", "
                //         << kalman.P(2, 2) << ", "
                //         << kalman.P(3, 3) << std::endl;

                // std::cerr << "imu: gyro: " << gyro[0] << ", " << gyro[1] << ", " << gyro[2];
                // std::cerr << "                                                 acc " << acc[0] << ", " << acc[1] << ", " << acc[2] << std::endl;
                // std::cerr << " time" << timestamp << std::endl;
            } else {
                std::cerr << "imu: msgpack read error : " << cmp_strerror(&cmp) << std::endl;
            }

        }
    }

    return 0;
}