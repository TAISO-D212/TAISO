package com.d212.taiso.domain.bookmark.service;

import com.d212.taiso.domain.bookmark.dto.BookmarkAddReq;
import com.d212.taiso.domain.bookmark.dto.BookmarkListRes;
import com.d212.taiso.domain.bookmark.entity.Bookmark;
import com.d212.taiso.domain.bookmark.repository.BookmarkRepository;
import com.d212.taiso.domain.member.entity.Member;
import com.d212.taiso.domain.place.entity.Place;
import com.d212.taiso.domain.place.repository.PlaceRepository;
import com.d212.taiso.global.result.error.ErrorCode;
import com.d212.taiso.global.result.error.exception.BusinessException;
import com.d212.taiso.global.util.CommonUtil;
import jakarta.transaction.Transactional;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.stereotype.Service;

import java.util.List;

@Log4j2
@Service
@RequiredArgsConstructor
public class BookmarkServiceImpl implements BookmarkService {

    private final BookmarkRepository bookmarkRepository;
    private final PlaceRepository placeRepository;

    private final CommonUtil commonUtil;

    // 해당 유저가 등록한 즐겨찾기 리스트 조회하기
    @Override
    public List<BookmarkListRes> getMyBookmarkList() {
        // 요청한 멤버의 정보 가져오기
        Member member = commonUtil.getMember();

        return bookmarkRepository.findBookmarkListResByMember(member);
    }

    @Override
    public void addBookmark(BookmarkAddReq bookmarkAddReq) {
        // 요청한 멤버의 정보 가져오기
        Member member = commonUtil.getMember();

        // 이게 맞냐??? (더 좋은 방식 알려줘!!!)
        Place place = Place.builder()
                .latitude(bookmarkAddReq.getLatitude())
                .longitude(bookmarkAddReq.getLongitude())
                .address(bookmarkAddReq.getAddress())
                .build();

        Bookmark bookmark = Bookmark.builder()
                .member(member)
                .name(bookmarkAddReq.getName())
                .place(place)
                .build();

        placeRepository.save(place);
        bookmarkRepository.save(bookmark);
    }

    @Override
    public void deleteBookmark(Long bookmarkId) {
        // 유저 인증도 해야되지 않나??
        // 100번이어도 성공했다하는데??
        // 이거 유저 확인이랑 번호도 하는 것으로 해서 예약 처리 ㄱ
        Member member = commonUtil.getMember();

        Bookmark bookmark = bookmarkRepository.findBookmarkByMemberAndAndId(member, bookmarkId)
                .orElseThrow(() -> new BusinessException(ErrorCode.BOOKMARK_NOT_EXIST));

        bookmarkRepository.delete(bookmark);

    }


}
