#include "mygcodefs.h"
#include "includes.h"

//static uint32_t date=0;
//static FILINFO  finfo;
//static uint16_t len = 0;
/*
SENDING:M20
Begin file list
PI3MK2~1.GCO 11081207
/YEST~1/TEST2/PI3MK2~1.GCO 11081207
/YEST~1/TEST2/PI3MK2~3.GCO 11081207
/YEST~1/TEST2/PI3MK2~2.GCO 11081207
/YEST~1/TEST2/PI3MK2~4.GCO 11081207
/YEST~1/TEST2/PI3MK2~5.GCO 11081207
/YEST~1/PI3MK2~1.GCO 11081207
/YEST~1/PI3MK2~3.GCO 11081207
/YEST~1/PI3MK2~2.GCO 11081207
End file list
*/
bool scanPrintFilesGcodeFs(void)
{
  clearInfoFile();

  char *ret = request_M20();
  char *data = malloc(strlen(ret) + 1);
  strcpy(data,ret);
  clearRequestCommandInfo();
  char s[3];

  strcpy(s, ",");

  data = strtok (data ,"]"); // обрезаем конец
  char *line = strtok(strchr(data ,'files":[')+1, s);
  for (;line != NULL;line = strtok(NULL, s))
  {
    // if (strcmp(line,"Begin file list") == 0 || strcmp(line,"End file list") == 0 || strcmp(line,"ok") == 0) continue; // Start and Stop tag
    // if (strlen(line) < strlen(infoFile.title)-4) continue; // No path line exclude
    if (strlen(infoFile.title) > 4 && strstr(line,infoFile.title+4) == NULL) continue; // No current directory

    char *pline = line + 1 + strlen(infoFile.title) - 4;
    if (strlen(infoFile.title) > 4)pline++;

    if (strchr(pline, '*') == NULL)
    {
      // FILE
      if (infoFile.f_num >= FILE_NUM)
        continue; /* Gcode max number is FILE_NUM*/

      char * Pstr_tmp = strrchr (line,'"');
      if (Pstr_tmp != NULL) *Pstr_tmp = 0; //remove правую ковычку
      Pstr_tmp = strrchr (line ,'"'); //remove начальная ковычка
      if (Pstr_tmp == NULL) Pstr_tmp = line;
      else Pstr_tmp++;
      infoFile.Longfile[infoFile.f_num]= malloc(strlen(Pstr_tmp) + 1);
      // if (infoFile.Longfile[infoFile.f_num] == NULL)
      // {
      //   clearRequestCommandInfo();
      //   break;
      // }
      strcpy(infoFile.Longfile[infoFile.f_num], Pstr_tmp);
      // clearRequestCommandInfo();  // for M33

      infoFile.file[infoFile.f_num] = malloc(strlen(pline) + 1);
      if (infoFile.file[infoFile.f_num] == NULL) break;
      strcpy(infoFile.file[infoFile.f_num++], pline);
    }
    else
    {
      // DIRECTORY
      if (infoFile.F_num >= FOLDER_NUM)
        continue; /* floder max number is FOLDER_NUM */

      char* rest = pline+1;
      char* folder = strtok_r(rest,"\"",&rest);

      bool found = false;
      for(int i=0; i < infoFile.F_num; i++)
      {
        if(strcmp(folder, infoFile.folder[i]) == 0)
        {
          found = true;
          break;
        }
      }

      if(!found)
      {
        uint16_t len = strlen(folder) + 1;
        infoFile.folder[infoFile.F_num] = malloc(len);
        if (infoFile.folder[infoFile.F_num] == NULL)
          break;
        strcpy(infoFile.folder[infoFile.F_num++], folder);

      }
    }
  }
  free(data);
  return true;
}
