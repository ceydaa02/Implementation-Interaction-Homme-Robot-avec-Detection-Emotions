#include <webots/robot.h>
#include <webots/skin.h>
#include <webots/console.h>
#include <string.h>
#include <stdio.h>

#define TIME_STEP 32

int main(int argc, char **argv) {
  wb_robot_init();

  // Argümanlardan karakter adını bul (örn. -d Robert)
  const char* character_name = NULL;
  for (int i = 1; i < argc - 1; ++i) {
    if (strcmp(argv[i], "-d") == 0) {
      character_name = argv[i + 1];
      break;
    }
  }

  if (character_name == NULL) {
    wb_console_print("[HATA] -d ile karakter ismi verilmemiş!\n", WB_STDERR);
    wb_robot_cleanup();
    return 1;
  }

  WbDeviceTag skin = wb_robot_get_device(character_name);
  if (skin == 0) {
    char msg[128];
    snprintf(msg, sizeof(msg), "[HATA] %s adlı karakterin Skin cihazı bulunamadı.\n", character_name);
    wb_console_print(msg, WB_STDERR);
    wb_robot_cleanup();
    return 1;
  }

  // Kolları indir
  int bone_count = wb_skin_get_bone_count(skin);
  for (int i = 0; i < bone_count; ++i) {
    const char* bone_name = wb_skin_get_bone_name(skin, i);

    if (strcmp(bone_name, "LeftArm") == 0) {
      double orientation[4] = {0.0, 0.0, 1.0, -1.57};  // sola -90°
      wb_skin_set_bone_orientation(skin, i, orientation, false);
    }

    if (strcmp(bone_name, "RightArm") == 0) {
      double orientation[4] = {0.0, 0.0, -1.0, -1.57}; // sağa -90°
      wb_skin_set_bone_orientation(skin, i, orientation, false);
    }
  }

  // Pozisyonu sabitlemek için birkaç adım ilerle
  for (int t = 0; t < 100; ++t)
    wb_robot_step(TIME_STEP);

  wb_robot_cleanup();
  return 0;
}
