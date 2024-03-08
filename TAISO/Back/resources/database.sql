CREATE DATABASE taiso;

USE taiso;

CREATE TABLE `Users` (
	`user_id`	varchar(20)	NOT NULL,
	`user_pwd`	varchar(20)	NOT NULL,
	`user_name`	varchar(20)	NOT NULL,
	`user_face_img`	varchar(2000)	NOT NULL	COMMENT '사진 파일 경로',
	`user_create_date`	datetime	NOT NULL	DEFAULT now(),
	`user_delete_date`	datetime	NULL,
   CONSTRAINT `PK_USERS` PRIMARY KEY (`user_id`)
)default character set utf8mb4;

CREATE TABLE `Reservations` (
	`rsv_id`	int	NOT NULL	AUTO_INCREMENT,
	`fk_rsv_user_id`	varchar(20)	NOT NULL,
	`fk_rsv_dest_id`	int	NOT NULL,
	`rsv_date`	datetime	NOT NULL,
	`rsv_stop_cnt`	int	NOT NULL	DEFAULT 1,
	`rsv_user_cnt`	int	NOT NULL	DEFAULT 1,
	`fk_rsv_bus_id`	int	NOT NULL,
	`rsv_arrival_time`	datetime	NULL,
	CONSTRAINT `PK_RESERVATIONS` PRIMARY KEY (`rsv_id`)
)default character set utf8mb4;

CREATE TABLE `Places` (
	`place_id`	int	NOT NULL AUTO_INCREMENT,
	`place_latitude`	double	NOT NULL,
	`place_longitude`	double	NOT NULL,
	CONSTRAINT `PK_PLACES` PRIMARY KEY (`place_id`)
)default character set utf8mb4;

CREATE TABLE `Rsv_details` (
	`rsv_id`	int	NOT NULL,
	`place_id`	int	NOT NULL,
	`fk_rd_user_id`	varchar(20)	NOT NULL,
	`rd_user_cnt`	int	NOT NULL	DEFAULT 1,
	`fd_depart_yn`	boolean	NOT NULL	DEFAULT false,
	`rd_order`	int	NULL,
	`rd_board_yn`	boolean	NOT NULL	DEFAULT false,
	`rd_stop_yn`	boolean	NOT NULL	DEFAULT FALSE,
	`rd_arrival_time`	DATETIME NULL,
	CONSTRAINT `PK_RSV_DETAILS` PRIMARY KEY (`rsv_id`,	`place_id`)
)default character set utf8mb4;

CREATE TABLE `Places_lists` (
	`pl_id`	int	NOT NULL	AUTO_INCREMENT,
	`pl_place_id`	int	NOT NULL,
	`pl_place_name`	varchar(20)	NOT NULL,
	`fk_pl_user_id`	varchar(20)	NULL	COMMENT 'null일 경우 공공장소',
	CONSTRAINT `PK_PLACES_LISTS` PRIMARY KEY (`pl_id`, `pl_place_id`)
)default character set utf8mb4;

CREATE TABLE `Rsv_routes` (
	`rsv_id`	int	NOT NULL,
	`rr_latitude`	double	NOT NULL,
	`rr_longitude`	double	NOT NULL,
	`rr_time`	datetime	NOT NULL	DEFAULT NOW(),
	CONSTRAINT `PK_RSV_ROUTES` PRIMARY KEY (`rsv_id`)
)default character set utf8mb4;

CREATE TABLE `Buses` (
	`bus_id`	int	NOT NULL	AUTO_INCREMENT,
	`bus_run_yn`	boolean	NOT NULL	DEFAULT false,
	`bus_user_cnt`	int	NOT NULL	DEFAULT 0,
	`bus_latitude`	double	NOT NULL,
	`bus_longitude`	double	NOT NULL,
	`bus_updated_date`	datetime	NOT NULL	DEFAULT now(),
	`bus_max_user`	int	NOT NULL	DEFAULT 4,
	CONSTRAINT `PK_BUSES` PRIMARY KEY (`bus_id`)
)default character set utf8mb4;

ALTER TABLE `Reservations` ADD CONSTRAINT `FK_Users_TO_Reservations_1` FOREIGN KEY (
	`fk_rsv_user_id`
)
REFERENCES `Users` (
	`user_id`
) ON DELETE cascade;

ALTER TABLE `Reservations` ADD CONSTRAINT `FK_Places_TO_Reservations_1` FOREIGN KEY (
	`fk_rsv_dest_id`
)
REFERENCES `Places` (
	`place_id`
) ON DELETE cascade;

ALTER TABLE `Reservations` ADD CONSTRAINT `FK_Buses_TO_Reservations_1` FOREIGN KEY (
	`fk_rsv_bus_id`
)
REFERENCES `Buses` (
	`bus_id`
);

ALTER TABLE `Rsv_details` ADD CONSTRAINT `FK_Reservations_TO_Rsv_details_1` FOREIGN KEY (
	`rsv_id`
)
REFERENCES `Reservations` (
	`rsv_id`
) ON DELETE cascade;

ALTER TABLE `Rsv_details` ADD CONSTRAINT `FK_Places_TO_Rsv_details_1` FOREIGN KEY (
	`place_id`
)
REFERENCES `Places` (
	`place_id`
) ON DELETE cascade;

ALTER TABLE `Rsv_details` ADD CONSTRAINT `FK_Users_TO_Rsv_details_1` FOREIGN KEY (
	`fk_rd_user_id`
)
REFERENCES `Users` (
	`user_id`
) ON DELETE cascade;

ALTER TABLE `Places_lists` ADD CONSTRAINT `FK_Places_TO_Places_lists_1` FOREIGN KEY (
	`pl_place_id`
)
REFERENCES `Places` (
	`place_id`
) ON DELETE cascade;

ALTER TABLE `Places_lists` ADD CONSTRAINT `FK_Users_TO_Places_lists_1` FOREIGN KEY (
	`fk_pl_user_id`
)
REFERENCES `Users` (
	`user_id`
) ON DELETE cascade;

ALTER TABLE `Rsv_routes` ADD CONSTRAINT `FK_Reservations_TO_Rsv_routes_1` FOREIGN KEY (
	`rsv_id`
)
REFERENCES `Reservations` (
	`rsv_id`
) ON DELETE cascade;

