#include <fstream>
#include <thread>
#include "Scene.hpp"
#include "Renderer.hpp"
#include <mutex>
#include <vector>

inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;
int g_complateTotals = 0;
std::mutex g_mutex;

void render_thread(std::vector<Vector3f>& fbuffer, const Scene& scene, int spp, int y0, int y1) {
    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    for (int i = y0; i < y1; i++) {
        for (int j = 0; j < scene.width; j++) {
            int index = i * scene.width + j;
            for (int k = 0; k < spp; k++) {
                float x = get_random_float();
                float y = get_random_float();
                float _x = (2 * (j + x) / (float)scene.width - 1) * imageAspectRatio * scale;
                float _y = (1 - 2 * (i + y) / (float)scene.height) * scale;
                Vector3f dir = normalize(Vector3f(-_x, _y, 1));
                Ray ray = Ray(eye_pos, dir);
                fbuffer[index] += scene.castRay(ray, 0) / spp;
            }
        }
        g_mutex.lock();
        g_complateTotals++;
        UpdateProgress(g_complateTotals / (float)scene.height);
        g_mutex.unlock();
    }
}

void Renderer::Render(const Scene& scene) {
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);

    const int thread_cnt = 12;
    int finished_thread = 0;
    int finished_width = 0;
    std::mutex mtx;

    int spp = 128;

    printf("%d %d\n", scene.height, scene.width);
    auto multiThreadCastRay = [&](uint32_t y_min, uint32_t y_max)
    {
        for (uint32_t j = y_min; j <= y_max; ++j) {
            for (uint32_t i = 0; i < scene.width; ++i) {
                // generate primary ray direction
                float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                        imageAspectRatio * scale;
                float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

                Vector3f dir = normalize(Vector3f(-x, y, 1));
                for (int k = 0; k < spp; k++) {
                    framebuffer[scene.width * j + i] += scene.castRay(Ray(eye_pos, dir), 0) / spp;
                }
            }
            //printf("%d\n", j);
            //UpdateProgress(j / (float)scene.height);
            mtx.lock();
            UpdateProgress(++finished_width * 1.0 / scene.width);
            mtx.unlock();
        }
    };
    int block = scene.height / thread_cnt + (scene.height % thread_cnt != 0);
    std::thread th[thread_cnt];
    for (int i = 0; i < thread_cnt; i++) {
        th[i] = std::thread(multiThreadCastRay, i * block, std::min((i + 1) * block - 1, scene.height));
    }
    for (int i = 0; i < thread_cnt; i++) th[i].join();
    UpdateProgress(1.0);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);
}