package com.d212.taiso.domain.place.repository;

import com.d212.taiso.domain.place.entity.Place;
import org.springframework.data.jpa.repository.JpaRepository;

import java.util.Optional;

public interface PlaceRepository extends JpaRepository<Place, Long> {

    // id를 통해 장소 찾기
    Optional<Place> findPlaceById(Long placeId);

}
