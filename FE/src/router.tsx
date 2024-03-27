import { createBrowserRouter } from 'react-router-dom';
import { LoginPage } from './pages/login/LoginPage.tsx';
import { SignUpPage } from './pages/signup/SignUpPage.tsx';
import { MainPage } from './pages/main/MainPage.tsx';
import { Reservation } from './pages/reservation/Reservation.tsx';
import { NewReservation } from './pages/reservation/NewReservation.tsx';
import { History } from './pages/history/History.tsx';
import { NotFound } from './pages/NotFound.tsx';
import { UserPage } from './pages/profile/UserPage.tsx';
import { SetDeparture } from './pages/main/components/SetDeparture.tsx';
import { SetArrival } from './pages/main/components/SetArrival.tsx';

export const router = createBrowserRouter([
	{ path: '/', element: <LoginPage /> },
	{ path: '/signup', element: <SignUpPage /> },
	{ path: '/main', element: <MainPage /> },
	{ path: '/reservation', element: <Reservation /> },
	{ path: '/reservation/new', element: <NewReservation /> },
	{ path: '/setDeparture', element: <SetDeparture /> },
	{ path: '/setArrival', element: <SetArrival /> },
	{ path: '/history', element: <History /> },
	// {path: '/favorite', element: <NotFound />},
	{ path: '/profile', element: <UserPage /> },
	{ path: '/*', element: <NotFound /> },
]);
