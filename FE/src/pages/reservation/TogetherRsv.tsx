import AccordionGroup from '@mui/joy/AccordionGroup';
import Accordion from '@mui/joy/Accordion';
import AccordionDetails, { accordionDetailsClasses } from '@mui/joy/AccordionDetails';
import AccordionSummary, { accordionSummaryClasses } from '@mui/joy/AccordionSummary';
import Stack from '@mui/joy/Stack';
import Typography from '@mui/joy/Typography';
import Avatar from '@mui/joy/Avatar';
import FormControl from '@mui/joy/FormControl';
import ListItemContent from '@mui/joy/ListItemContent';
import DirectionsBusIcon from '@mui/icons-material/DirectionsBus';
import PeopleAltIcon from '@mui/icons-material/PeopleAlt';
import { HomeBackButton } from '../../components/HomeBackButton';
import { useNavigate, useParams } from 'react-router-dom';
import { FaBusSimple } from 'react-icons/fa6';
import 'dayjs/locale/ko';
import { useEffect, useState } from 'react';
import Box from '@mui/material/Box';
import InputLabel from '@mui/material/InputLabel';
import MenuItem from '@mui/material/MenuItem';
import Select, { SelectChangeEvent } from '@mui/material/Select';
import { addTogetherRsv } from '../../apis/reservationApi';
import TogetherRsvStore from '../../store/TogetherRsvStore';
import { TogetherRsvInputType } from '../../interfaces/Reservation';

export const TogetherRsv = () => {

	const { rsvId } = useParams(); // 문자열로 받아짐.
	const rsvIdInt = parseInt(rsvId, 10);

	const { setBookmarkId, setLatitude, setLongitude, setAddress, setCnt } = TogetherRsvStore();

	const [rsvObj, setRsvObj] = useState<TogetherRsvInputType>({
		bookmarkId: null,
		latitude: null,
		longitude: null,
		address: null,
		cnt: 1,
	});


	const [userCnt, setUserCnt] = useState<number>(1);

	const navigate = useNavigate();

  // 로컬스토리지로 부터 데이터 가져오기
	const bookmarkId = JSON.parse(localStorage?.getItem('TogetherRsv')).state?.bookmarkId;
	const latitude = JSON.parse(localStorage?.getItem('TogetherRsv')).state?.latitude;
	const longitude = JSON.parse(localStorage?.getItem('TogetherRsv')).state?.longitude;
	const address = JSON.parse(localStorage?.getItem('TogetherRsv')).state?.address;
	const cnt = JSON.parse(localStorage?.getItem('TogetherRsv')).state?.cnt;

	useEffect(() => {
		setRsvObj({
			bookmarkId,
			latitude,
			longitude,
			address,
			cnt,
		});
	}, [address, bookmarkId, cnt, latitude, longitude]);

	const goSetTogetherDeparture = (rsvId:number) => {
		navigate(`/setTogetherDeparture/${rsvId}`);
	};

	const handleCntChange = (event: SelectChangeEvent) => {
		setUserCnt(event.target.value as unknown as number);
		setCnt(event.target.value as unknown as number);
	};

	const submitTogetherRsv = () => {
		console.log(rsvObj);
		addTogetherRsv(rsvIdInt, rsvObj).then((res) => {
			console.log(res);
			setBookmarkId(null);
			setLatitude(null);
			setLongitude(null);
			setAddress(null);
			setCnt(null);
			navigate('/history', { replace: true });
		});
	};

	return (
		<>
			<div className='fixed z-10 top-0 w-[100%] h-[15%] flex-col justify-evenly items-end bg-white'>
				<div className='flex'>
					<HomeBackButton />
					<div className='fixed flex justify-center w-[100%] top-[3%] mx-[5px] font-["Pretendard-Bold"] text-[25px]'>
						<div>장소 찾기</div>
					</div>
				</div>
			</div>
			<div className='fixed top-[15%] w-[100%] h-[70%] flex justify-center bg-white overflow-y-scroll'>
				<AccordionGroup
					variant='plain'
					transition='0.2s'
					sx={{
						maxWidth: 400,
						borderRadius: 'md',
						[`& .${accordionDetailsClasses.content}.${accordionDetailsClasses.expanded}`]: {
							paddingBlock: '1rem',
						},
						[`& .${accordionSummaryClasses.button}`]: {
							paddingBlock: '1rem',
						},
					}}>
					<Accordion>
						<AccordionSummary>
							<Avatar color='primary'>
								<DirectionsBusIcon />
							</Avatar>
							<ListItemContent>
								<Typography level='title-md'>장소 선택</Typography>
								<Typography level='body-sm'>원하는 출발지 도착지를 설정하세요.</Typography>
							</ListItemContent>
						</AccordionSummary>
						<AccordionDetails>
							<Stack spacing={1.5}>
								<FormControl orientation='horizontal' sx={{ gap: 3 }}>
									<div className='w-[100%] h-[100%] py-2 bg-white border-2 border-[#C4B5FC] rounded-md divide-y divide-dashed divide-[#C4B5FC]'>
										<div className='h-[100%] flex items-center' onClick={()=> goSetTogetherDeparture(rsvIdInt)}>
											<span className='mx-[5px]'>
												<FaBusSimple color='navy' className='mx-[5px]' />
											</span>
											<span className='mx-[5px]'>출발지 : {address}</span>
										</div>
									</div>
								</FormControl>

                {/* 나중에 도착지 정보를 담을 수 있다면. */}

								{/* <FormControl orientation='horizontal' sx={{ gap: 1 }}>
									<div className='w-[100%] h-[100%] py-2 bg-white border-2 border-[#C4B5FC] rounded-md divide-y divide-dashed divide-[#C4B5FC]'>
										<div className='h-[100%] flex items-center' onClick={goSetArrival}>
											<span className='mx-[5px]'>
												<FaBusSimple color='skyblue' className='mx-[5px]' />
											</span>
											<span className='mx-[5px]'>목적지 : {endAddress}</span>
											<span></span>
										</div>
									</div>
								</FormControl> */}
							</Stack>
						</AccordionDetails>
					</Accordion>

					<Accordion>
						<AccordionSummary>
							<Avatar color='neutral'>
								<PeopleAltIcon />
							</Avatar>
							<ListItemContent>
								<Typography level='title-md'>인원 선택</Typography>
								<Typography level='body-sm'>나를 포함한 인원을 선택하세요.</Typography>
							</ListItemContent>
						</AccordionSummary>
						<AccordionDetails>
							<Box sx={{ width: '100%', marginTop: '3%' }}>
								<FormControl className='w-[100%]'>
									<InputLabel id='userCnt'>총 탑승 인원</InputLabel>
									<Select id='userCnt' value={userCnt} onChange={handleCntChange}>
										<MenuItem value={1}>1 명</MenuItem>
										<MenuItem value={2}>2 명</MenuItem>
										<MenuItem value={3}>3 명</MenuItem>
										<MenuItem value={4}>4 명</MenuItem>
									</Select>
								</FormControl>
							</Box>
						</AccordionDetails>
					</Accordion>
				</AccordionGroup>
			</div>
			<div className='fixed z-10 bottom-0 w-[100%] h-[15%] flex flex-col justify-center items-center bg-white'>
				<div className=' w-[70%] h-[40%] flex justify-center items-center bg-[#3422F2] rounded-full'>
					<span
						className='flex justify-center w-[100%] font-["Pretendard-Bold"] text-[25px] text-white'
						onClick={submitTogetherRsv}>
						예약하기
					</span>
				</div>
			</div>
		</>
	);
};
