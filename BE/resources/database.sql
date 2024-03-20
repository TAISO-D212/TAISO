CREATE DATABASE IF NOT EXISTS TAISO;

USE TAISO;

CREATE TABLE IF NOT EXISTS `Member` (
	`email`	varchar(20)	NOT NULL,
	`pwd`	varchar(200)	NOT NULL,
	`name`	varchar(20)	NOT NULL,
	`face_img`	varchar(2000)	NOT NULL	COMMENT '사진 파일 경로',
	`create_date`	datetime	NOT NULL	DEFAULT now(),
	`delete_flag`	boolean	NOT NULL	DEFAULT false,
   CONSTRAINT `PK_Member` PRIMARY KEY (`email`)
)default character set utf8mb4;

CREATE TABLE IF NOT EXISTS `Reservation` (
	`rsv_id`	int	NOT NULL	AUTO_INCREMENT,
	`email`	varchar(20)	NOT NULL,
	`place_id`	int	NOT NULL,
	`time`	datetime	NOT NULL,
	`stop_cnt`	int	NOT NULL	DEFAULT 1,
	`cnt`	int	NOT NULL	DEFAULT 1,
	`arrival_time`	datetime	NULL,
	`route_img`	varchar(2000)	NULL,
	CONSTRAINT `PK_RESERVATION` PRIMARY KEY (`rsv_id`)
)default character set utf8mb4;

CREATE TABLE IF NOT EXISTS `Place` (
	`place_id`	int	NOT NULL AUTO_INCREMENT,
	`latitude`	double	NOT NULL,
	`longitude`	double	NOT NULL,
	`address`	varchar(200)	NOT NULL,
	CONSTRAINT `PK_PLACE` PRIMARY KEY (`place_id`)
)default character set utf8mb4;

CREATE TABLE IF NOT EXISTS `Rsv_detail` (
	`rsv_id`	int	NOT NULL,
	`place_id`	int	NOT NULL,
	`email`	varchar(20)	NOT NULL,
	`cnt`	int	NOT NULL	DEFAULT 1,
	`order`	int	NULL,
	`depart_flag`	boolean	NOT NULL	DEFAULT false,
	`board_flag`	boolean	NOT NULL	DEFAULT false,
	`stop_flag`	boolean	NOT NULL	DEFAULT FALSE,
	`arrival_time`	DATETIME NULL,
	CONSTRAINT `PK_RSV_DETAIL` PRIMARY KEY (`rsv_id`,	`place_id`)
)default character set utf8mb4;

CREATE TABLE IF NOT EXISTS `Places_list` (
	`pl_id`	int	NOT NULL	AUTO_INCREMENT,
	`place_id`	int	NOT NULL,
	`name`	varchar(20)	NOT NULL,
	`email`	varchar(20)	NULL	COMMENT 'null일 경우 공공장소',
	CONSTRAINT `PK_PLACES_LIST` PRIMARY KEY (`pl_id`, `place_id`)
)default character set utf8mb4;

CREATE TABLE IF NOT EXISTS `Rsv_route` (
	`rsv_id`	int	NOT NULL,
	`latitude`	double	NOT NULL,
	`longitude`	double	NOT NULL,
	`time`	datetime	NOT NULL	DEFAULT NOW(),
	CONSTRAINT `PK_RSV_ROUTE` PRIMARY KEY (`rsv_id`)
)default character set utf8mb4;


ALTER TABLE `Reservation` ADD CONSTRAINT `FK_Member_TO_Reservation_1` FOREIGN KEY (
	`email`
)
REFERENCES `Member` (
	`email`
) on delete cascade;

ALTER TABLE `Reservation` ADD CONSTRAINT `FK_Place_TO_Reservation_1` FOREIGN KEY (
	`place_id`
)
REFERENCES `Place` (
	`place_id`
) on delete cascade;

ALTER TABLE `Rsv_detail` ADD CONSTRAINT `FK_Reservation_TO_Rsv_detail_1` FOREIGN KEY (
	`rsv_id`
)
REFERENCES `Reservation` (
	`rsv_id`
) on delete cascade;

ALTER TABLE `Rsv_detail` ADD CONSTRAINT `FK_Place_TO_Rsv_detail_1` FOREIGN KEY (
	`place_id`
)
REFERENCES `Place` (
	`place_id`
) on delete cascade;

ALTER TABLE `Rsv_detail` ADD CONSTRAINT `FK_Member_TO_Rsv_detail_1` FOREIGN KEY (
	`email`
)
REFERENCES `Member` (
	`email`
) on delete cascade;

ALTER TABLE `Places_list` ADD CONSTRAINT `FK_Place_TO_Places_list_1` FOREIGN KEY (
	`place_id`
)
REFERENCES `Place` (
	`place_id`
) on delete cascade;

ALTER TABLE `Places_list` ADD CONSTRAINT `FK_Member_TO_Places_list_1` FOREIGN KEY (
	`email`
)
REFERENCES `Member` (
	`email`
) on delete cascade;

ALTER TABLE `Rsv_route` ADD CONSTRAINT `FK_Reservation_TO_Rsv_route_1` FOREIGN KEY (
	`rsv_id`
)
REFERENCES `Reservation` (
	`rsv_id`
) on delete cascade;

