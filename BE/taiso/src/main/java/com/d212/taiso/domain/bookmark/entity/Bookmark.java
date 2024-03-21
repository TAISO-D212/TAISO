package com.d212.taiso.domain.bookmark.entity;

import static jakarta.persistence.FetchType.*;

import com.d212.taiso.domain.member.entity.Member;
import com.d212.taiso.domain.place.entity.Place;
import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.GeneratedValue;
import jakarta.persistence.GenerationType;
import jakarta.persistence.Id;
import jakarta.persistence.JoinColumn;
import jakarta.persistence.ManyToOne;
import lombok.AccessLevel;
import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NoArgsConstructor;

/**
 * Created by 전근렬 on 2024-03-21
 */
@Entity
@Getter
@AllArgsConstructor
@NoArgsConstructor(access = AccessLevel.PROTECTED)
public class Bookmark {
    
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name = "bookmark_id")
    private Long id;

    @ManyToOne(fetch = LAZY)
    @JoinColumn(name = "email")
    private Member member;

    @ManyToOne(fetch = LAZY)
    @JoinColumn(name = "place_id")
    private Place place;

    private String name; // 장소 이름
}
