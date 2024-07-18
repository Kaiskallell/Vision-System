/*
 * Created on Tue Dec 10 2019
 *
 * Copyright (c) 2019 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential!
 */

#ifndef __VS_C_COMMUNICATION_ERROR_H__
#define __VS_C_COMMUNICATION_ERROR_H__

/* Transferred via Nack.proto message to ST/PS
 * - enum has to be exchanged with Lachmann & Rink after changing.
 */
typedef enum
{
	PB_ERR_NONE = 0x00000000,  // not possible

	// common errors
	PB_ERR_PARSING = 0x00001001,             // Steuerungsnachricht kann nicht entpackt werden
	PB_ERR_UNKNOWN_COMMAND_ID = 0x00001002,  // Unbekannte Kommandokennung in Steuerungsnachricht
	PB_ERR_SEQUENCE_INVALID = 0x00001003,    // Ungueltige Sequenzenummer in Steuerungsnachricht

	// VS error from 0x00001101
	VS_ERR_WRONG_VERSION = 0x00001101,    // Falsche Protokollversion der Steuerungsnachricht
	VS_ERR_NO_PROGRAM_LIST = 0x00001102,  // keine Vision System Programme installiert
	VS_ERR_NO_CAMERA_SYSTEM_CONNECTED =
	    0x00001103,  // Verbindungsfehler Bildverarbeitungsystem - keine Kamera angeschlossen
	VS_ERR_UNKNOWN_DATABASE_ENTRY = 0x00001104,  // Datenbankeintrag fehlt im Vision System
	VS_ERR_UNKNOWN_APP_ID = 0x00001105,          // Angefragte Formatnummer ist unbekannt
	VS_ERR_WRONG_ROBOT_TYPE = 0x00001106,        // Steuerungsnachricht inkompatibel mit Robotertyp
	VS_ERR_FORMAT_FILE_NOT_FOUND = 0x00001107,   // Format auf Vision System nicht gefunden

	VS_ERR_FORMAT_GENERALPARAMETERS_PICK = 0x00001108, // General Format parameters könnten nicht gelesen werden für die Aufnahme Kamera
	VS_ERR_FORMAT_GENERALPARAMETERS_PLACE = 0x00001109, // General Format parameters könnten nicht gelesen werden für die Ablage Kamera
	VS_ERR_FORMAT_GENERALSTECHTESTPARAMETERS = 0x00001110, // General Stechtest parameters könnten nicht gelesen werden
	VS_ERR_FORMAT_GENERALPARAMETERS_CLASSIFICATOR = 0x00001111, // General Format parameters könnten nicht gelesen werden für den Klassifikator
	VS_ERR_UDPHANDLER_ERROR = 0x00001112, // UDP könnte nicht konfiguriert werden
	VS_ERR_CAMERA_INIT_ERROR_PICK = 0x00001113, // Aufnahme Kamera könnte nicht konfiguriert werden
	VS_ERR_CAMERA_INIT_ERROR_PLACE = 0x00001114, // Ablage Kamera könnte nicht konfiguriert werden
	VS_ERR_CAMERA_INIT_ERROR_STECHTEST = 0x00001115, // Stechtest Kamera könnte nicht konfiguriert werden
	VS_ERR_CAMERA_INIT_ERROR_CLASSIFICATOR = 0x00001116, // Klassifikator Kamera könnte nicht konfiguriert werden
	VS_ERR_DETECTOR_WRONGPARAMETERS_PICK = 0x00001117, // Objekt detektor für die Aufnahme könnte nicht konfiguriert werden
	VS_ERR_DETECTOR_WRONGPARAMETERS_PLACE = 0x00001118, // Objekt detektor für die Ablage könnte nicht konfiguriert werden
	VS_ERR_DETECTOR_WRONGPARAMETERS_CLASSIFICATOR = 0x00001119, // Objekt detektor für den Klassifikator könnte nicht konfiguriert werden
	VS_ERR_CAMERA_EMPTYIMAGE_PICK = 0x00001120, // Das Bild von der Aufnahme Kamera ist leer
	VS_ERR_CAMERA_EMPTYIMAGE_PLACE = 0x00001121, // Das Bild von der Ablage Kamera ist leer
	VS_ERR_CAMERA_EMPTYIMAGE_CLASSIFICATOR = 0x00001122, // Das Bild von der Klassifikator Kamera ist leer
	VS_ERR_CAMERA_EMPTYIMAGE_STECHTEST = 0x00001123, // Das Bild von der Stechtest Kamera ist leer
	VS_ERR_POSEESTIMATION_ERROR = 0x00001124, // Pose Estimation konnte nicht durchgeführt werden
	// NRT errors from 0x00001501

	PB_ERR = 0x00002000,  // maximum error number
} eResponseNACK_Errors;

#endif
