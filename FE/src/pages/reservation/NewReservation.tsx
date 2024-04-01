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
import CalendarMonthIcon from '@mui/icons-material/CalendarMonth';
import AccessTimeIcon from '@mui/icons-material/AccessTime';
import PeopleAltIcon from '@mui/icons-material/PeopleAlt';
import { BackButton } from '../../components/BackButton';
import { useNavigate } from 'react-router-dom';
import { FaBusSimple } from 'react-icons/fa6';
import { AdapterDayjs } from '@mui/x-date-pickers/AdapterDayjs';
import { LocalizationProvider } from '@mui/x-date-pickers/LocalizationProvider';
import { DateCalendar } from '@mui/x-date-pickers/DateCalendar';
import 'dayjs/locale/ko';
import { useEffect, useState } from 'react';
import dayjs, { Dayjs } from 'dayjs';
import ToggleButton from '@mui/material/ToggleButton';
import ToggleButtonGroup from '@mui/material/ToggleButtonGroup';
import Box from '@mui/material/Box';
import InputLabel from '@mui/material/InputLabel';
import MenuItem from '@mui/material/MenuItem';
import Select, { SelectChangeEvent } from '@mui/material/Select';
import NewReservationStore from '../../store/NewReservationStore';
import { addRsv } from '../../apis/reservationApi';
import { RsvInputType } from '../../interfaces/Reservation';

export const NewReservation = () => {
	const {
		setStartBookmarkId,
		setStartLatitude,
		setStartLongitude,
		setStartAddress,
		setEndBookmarkId,
		setEndLatitude,
		setEndLongitude,
		setEndAddress,
		setTime,
		setCnt,
	} = NewReservationStore();
	const [rsvObj, setRsvObj] = useState<RsvInputType>({
		startBookmarkId: null,
		startLatitude: null,
		startLongitude: null,
		startAddress: null,
		endBookmarkId: null,
		endLatitude: null,
		endLongitude: null,
		endAddress: null,
		time: '',
		cnt: 0,
	});
	const [date, setDate] = useState<Dayjs | null | undefined>(dayjs());
	const [daytime, setDaytime] = useState<string>('오전');
	const [rsvTime, setRsvTime] = useState<string>('');
	const [userCnt, setUserCnt] = useState<number>(1);
	const navigate = useNavigate();

	const startBookmarkId = JSON.parse(localStorage.getItem('NewReservation')).state?.startPlaceId;
	const startLatitude = JSON.parse(localStorage.getItem('NewReservation')).state?.startLatitude;
	const startLongitude = JSON.parse(localStorage.getItem('NewReservation')).state?.startLongitude;
	const startAddress = JSON.parse(localStorage.getItem('NewReservation')).state?.startAddress;
	const endBookmarkId = JSON.parse(localStorage.getItem('NewReservation')).state?.endPlaceId;
	const endLatitude = JSON.parse(localStorage.getItem('NewReservation')).state?.endLatitude;
	const endLongitude = JSON.parse(localStorage.getItem('NewReservation')).state?.endLongitude;
	const endAddress = JSON.parse(localStorage.getItem('NewReservation')).state?.endAddress;
	const time = JSON.parse(localStorage.getItem('NewReservation')).state?.time;
	const cnt = JSON.parse(localStorage.getItem('NewReservation')).state?.cnt;

	useEffect(() => {
		setRsvObj({
			startBookmarkId,
			startLatitude,
			startLongitude,
			startAddress,
			endBookmarkId,
			endLatitude,
			endLongitude,
			endAddress,
			time,
			cnt,
		});
	}, [rsvTime, userCnt]);

	const goSetDeparture = () => {
		navigate('/setDeparture');
	};

	const goSetArrival = () => {
		navigate('/setArrival');
	};

	const handleDaytimeChange = (event: React.MouseEvent<HTMLElement>, newDaytime: string) => {
		setDaytime(newDaytime);
	};

	const handleRsvTimeChange = (event: SelectChangeEvent) => {
		console.log(date?.format('YYYY-MM-DD') + 'T' + event.target.value);
		setRsvTime(event.target.value as string);
		if (date) {
			setTime(date?.format('YYYY-MM-DD') + 'T' + event.target.value);
		}
	};

	const handleCntChange = (event: SelectChangeEvent) => {
		setUserCnt(event.target.value as unknown as number);
		setCnt(event.target.value as unknown as number);
	};

	const submitNewRsv = () => {
		console.log(rsvObj);
		addRsv(rsvObj).then((res) => {
			console.log(res);
			setStartBookmarkId(null);
			setStartLatitude(null);
			setStartLongitude(null);
			setStartAddress(null);
			setEndBookmarkId(null);
			setEndLatitude(null);
			setEndLongitude(null);
			setEndAddress(null);
			setTime(null);
			setCnt(null);
			// navigate('/reservation');
		});
	};

	return (
		<>
			<div className='fixed z-10 top-0 w-[100%] h-[15%] flex-col justify-evenly items-end bg-white'>
				<div className='flex'>
					<BackButton />
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
								<FormControl orientation='horizontal' sx={{ gap: 1 }}>
									<div className='w-[100%] h-[100%] bg-white border-2 border-[#C4B5FC] rounded-lg divide-y divide-dashed divide-[#C4B5FC]'>
										<div className='h-[100%] flex items-center' onClick={goSetDeparture}>
											<span className='mx-[5px]'>
												<FaBusSimple color='navy' className='mx-[5px]' />
											</span>
											<span className='mx-[5px]'>출발지 : {startAddress}</span>
										</div>
									</div>
								</FormControl>

								<FormControl orientation='horizontal' sx={{ gap: 1 }}>
									<div className='w-[100%] h-[100%] bg-white border-2 border-[#C4B5FC] rounded-lg divide-y divide-dashed divide-[#C4B5FC]'>
										<div className='h-[100%] flex items-center' onClick={goSetArrival}>
											<span className='mx-[5px]'>
												<FaBusSimple color='skyblue' className='mx-[5px]' />
											</span>
											<span className='mx-[5px]'>목적지 : {endAddress}</span>
											<span></span>
										</div>
									</div>
								</FormControl>
							</Stack>
						</AccordionDetails>
					</Accordion>

					<Accordion>
						<AccordionSummary>
							<Avatar color='success'>
								<CalendarMonthIcon />
							</Avatar>
							<ListItemContent>
								<Typography level='title-md'>날짜 선택</Typography>
								<Typography level='body-sm'>이동하고 싶은 날을 선택하세요.</Typography>
							</ListItemContent>
						</AccordionSummary>
						<AccordionDetails>
							<LocalizationProvider dateAdapter={AdapterDayjs} adapterLocale='ko'>
								{/* <DateCalendar value={value} onChange={(newValue) => console.log(newValue.format())} /> */}
								<DateCalendar value={date} onChange={(newValue) => setDate(newValue)} />
							</LocalizationProvider>
						</AccordionDetails>
					</Accordion>

					<Accordion>
						<AccordionSummary>
							<Avatar color='warning'>
								<AccessTimeIcon />
							</Avatar>
							<ListItemContent>
								<Typography level='title-md'>시간 선택</Typography>
								<Typography level='body-sm'>원하는 시간을 선택하세요.</Typography>
							</ListItemContent>
						</AccordionSummary>
						<AccordionDetails>
							<ToggleButtonGroup
								sx={{ width: '100%' }}
								color='primary'
								value={daytime}
								exclusive
								onChange={handleDaytimeChange}>
								<ToggleButton sx={{ width: '50%' }} value='오전'>
									오전
								</ToggleButton>

								<ToggleButton sx={{ width: '50%' }} value='오후'>
									오후
								</ToggleButton>
							</ToggleButtonGroup>
							{daytime === '오전' ? (
								<Box sx={{ width: '100%', marginTop: '5%' }}>
									<FormControl className='w-[100%]'>
										<InputLabel id='am'>오전 시간 선택</InputLabel>
										<Select id='am' value={rsvTime} onChange={handleRsvTimeChange}>
											<MenuItem value={'00:00:00'}>00 : 00 ~ 01 : 00</MenuItem>
											<MenuItem value={'01:00:00'}>01 : 00 ~ 02 : 00</MenuItem>
											<MenuItem value={'02:00:00'}>02 : 00 ~ 03 : 00</MenuItem>
											<MenuItem value={'03:00:00'}>03 : 00 ~ 04 : 00</MenuItem>
											<MenuItem value={'04:00:00'}>04 : 00 ~ 05 : 00</MenuItem>
											<MenuItem value={'05:00:00'}>05 : 00 ~ 06 : 00</MenuItem>
											<MenuItem value={'06:00:00'}>06 : 00 ~ 07 : 00</MenuItem>
											<MenuItem value={'07:00:00'}>07 : 00 ~ 08 : 00</MenuItem>
											<MenuItem value={'08:00:00'}>08 : 00 ~ 09 : 00</MenuItem>
											<MenuItem value={'09:00:00'}>09 : 00 ~ 10 : 00</MenuItem>
											<MenuItem value={'10:00:00'}>10 : 00 ~ 11 : 00</MenuItem>
											<MenuItem value={'11:00:00'}>11 : 00 ~ 12 : 00</MenuItem>
										</Select>
									</FormControl>
								</Box>
							) : (
								<Box sx={{ width: '100%', marginTop: '5%' }}>
									<FormControl className='w-[100%]'>
										<InputLabel id='pm'>오후 시간 선택</InputLabel>
										<Select id='pm' value={rsvTime} onChange={handleRsvTimeChange}>
											<MenuItem value={'12:00:00'}>12 : 00 ~ 13 : 00</MenuItem>
											<MenuItem value={'13:00:00'}>13 : 00 ~ 14 : 00</MenuItem>
											<MenuItem value={'14:00:00'}>14 : 00 ~ 15 : 00</MenuItem>
											<MenuItem value={'15:00:00'}>15 : 00 ~ 16 : 00</MenuItem>
											<MenuItem value={'16:00:00'}>16 : 00 ~ 17 : 00</MenuItem>
											<MenuItem value={'17:00:00'}>17 : 00 ~ 18 : 00</MenuItem>
											<MenuItem value={'18:00:00'}>18 : 00 ~ 19 : 00</MenuItem>
											<MenuItem value={'19:00:00'}>19 : 00 ~ 20 : 00</MenuItem>
											<MenuItem value={'20:00:00'}>20 : 00 ~ 21 : 00</MenuItem>
											<MenuItem value={'21:00:00'}>21 : 00 ~ 22 : 00</MenuItem>
											<MenuItem value={'22:00:00'}>22 : 00 ~ 23 : 00</MenuItem>
											<MenuItem value={'23:00:00'}>23 : 00 ~ 24 : 00</MenuItem>
										</Select>
									</FormControl>
								</Box>
							)}
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
						onClick={submitNewRsv}>
						예약하기
					</span>
				</div>
			</div>
		</>
	);
};
