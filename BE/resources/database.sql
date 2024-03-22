CREATE DATABASE IF NOT EXISTS TAISO;
USE TAISO;

CREATE TABLE IF NOT EXISTS `member` (
	`email`	varchar(20)	NOT NULL,
	`pwd`	varchar(200)	NOT NULL,
	`name`	varchar(20)	NOT NULL,
	`face_img`	varchar(2000)	NULL	COMMENT '사진 파일 경로',
	`create_date`	datetime	NOT NULL	DEFAULT now(),
	`delete_flag`	boolean	NOT NULL	DEFAULT false,
   CONSTRAINT `PK_Member` PRIMARY KEY (`email`)
)default character set utf8mb4;

CREATE TABLE IF NOT EXISTS `reservation` (
	`rsv_id`	bigint	NOT NULL	AUTO_INCREMENT,
	`email`	varchar(20)	NOT NULL,
	`place_id`	bigint	NOT NULL,
	`time`	datetime	NOT NULL,
	`stop_cnt`	int	NOT NULL	DEFAULT 1,
	`cnt`	int	NOT NULL	DEFAULT 1,
	`arrival_time`	datetime	NULL,
	`route_img`	varchar(2000)	NULL,
	`route_dist`	varchar(200)	NULL,
	CONSTRAINT `PK_RESERVATION` PRIMARY KEY (`rsv_id`)
)default character set utf8mb4;

CREATE TABLE IF NOT EXISTS `place` (
	`place_id`	bigint	NOT NULL AUTO_INCREMENT,
	`latitude`	double	NOT NULL,
	`longitude`	double	NOT NULL,
	`address`	varchar(200)	NOT NULL,
	CONSTRAINT `PK_PLACE` PRIMARY KEY (`place_id`)
)default character set utf8mb4;

CREATE TABLE IF NOT EXISTS `rsv_detail` (
	`rsv_id`	bigint	NOT NULL,
	`place_id`	bigint	NOT NULL,
	`email`	varchar(20)	NOT NULL,
	`cnt`	int	NOT NULL	DEFAULT 1,
	`orders`	int	NULL,
	`depart_flag`	boolean	NOT NULL	DEFAULT false,
	`board_flag`	boolean	NOT NULL	DEFAULT false,
	`stop_flag`	boolean	NOT NULL	DEFAULT FALSE,
	`arrival_time`	DATETIME NULL,
	CONSTRAINT `PK_RSV_DETAIL` PRIMARY KEY (`rsv_id`,	`place_id`)
)default character set utf8mb4;

CREATE TABLE IF NOT EXISTS `bookmark` (
	`bookmark_id`	bigint	NOT NULL	AUTO_INCREMENT,
	`place_id`	bigint	NOT NULL,
	`name`	varchar(20)	NOT NULL,
	`email`	varchar(20)	NULL	COMMENT 'null일 경우 공공장소',
	CONSTRAINT `PK_BOOKMARK` PRIMARY KEY (`bookmark_id`)
)default character set utf8mb4;

CREATE TABLE IF NOT EXISTS `rsv_route` (
	`rsv_id`	bigint	NOT NULL,
	`latitude`	double	NOT NULL,
	`longitude`	double	NOT NULL,
	`time`	datetime	NOT NULL	DEFAULT NOW(),
	CONSTRAINT `PK_RSV_ROUTE` PRIMARY KEY (`rsv_id`)
)default character set utf8mb4;


ALTER TABLE `reservation` ADD CONSTRAINT `FK_Member_TO_Reservation_1` FOREIGN KEY (
	`email`
)
REFERENCES `member` (
	`email`
) on delete cascade;

ALTER TABLE `reservation` ADD CONSTRAINT `FK_Place_TO_Reservation_1` FOREIGN KEY (
	`place_id`
)
REFERENCES `place` (
	`place_id`
) on delete cascade;

ALTER TABLE `rsv_detail` ADD CONSTRAINT `FK_Reservation_TO_Rsv_detail_1` FOREIGN KEY (
	`rsv_id`
)
REFERENCES `reservation` (
	`rsv_id`
) on delete cascade;

ALTER TABLE `rsv_detail` ADD CONSTRAINT `FK_Place_TO_Rsv_detail_1` FOREIGN KEY (
	`place_id`
)
REFERENCES `place` (
	`place_id`
) on delete cascade;

ALTER TABLE `rsv_detail` ADD CONSTRAINT `FK_Member_TO_Rsv_detail_1` FOREIGN KEY (
	`email`
)
REFERENCES `member` (
	`email`
) on delete cascade;

ALTER TABLE `bookmark` ADD CONSTRAINT `FK_Place_TO_Bookmark_1` FOREIGN KEY (
	`place_id`
)
REFERENCES `place` (
	`place_id`
) on delete cascade;

ALTER TABLE `bookmark` ADD CONSTRAINT `FK_Member_TO_Bookmark_1` FOREIGN KEY (
	`email`
)
REFERENCES `member` (
	`email`
) on delete cascade;

ALTER TABLE `rsv_route` ADD CONSTRAINT `FK_Reservation_TO_Rsv_route_1` FOREIGN KEY (
	`rsv_id`
)
REFERENCES `reservation` (
	`rsv_id`
) on delete cascade;

