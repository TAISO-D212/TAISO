package com.d212.taiso.domain.bookmark.repository;

import com.d212.taiso.domain.bookmark.dto.BookmarkListRes;
import com.d212.taiso.domain.bookmark.entity.Bookmark;
import com.d212.taiso.domain.member.entity.Member;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;

import java.util.List;
import java.util.Optional;

public interface BookmarkRepository extends JpaRepository<Bookmark, Long> {

    // 해당 멤버에 대한 즐겨찾기(장소) 목록 가져오기

    @Query("select " +
            "new com.d212.taiso.domain.bookmark.dto.BookmarkListRes(b.name ,b.place) " +
            "from " +
            "Bookmark b " +
            "where b.member = :member ")
    List<BookmarkListRes> findBookmarkListResByMember(Member member);

    Optional<Bookmark> findBookmarkByMemberAndAndId(Member member, Long bookmarkId);
}
